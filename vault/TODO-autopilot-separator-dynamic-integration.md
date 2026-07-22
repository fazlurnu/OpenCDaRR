# Aircraft Dynamic Model Architecture

I came to a realization that what we've done is a bit incorrect. Let's start the aircraft model from scratch and define three vehicle categories:

- **multirotor**
- **fixed-wing**
- **VTOL**

The key architectural change is to separate:

1. **Mission / intent**
2. **Autopilot / guidance**
3. **Dynamics**

The simulator should not combine these responsibilities.

---

# Overall Architecture

```
                 Mission
                    |
                    v
            Autopilot / Guidance
                    |
                    v
             MotionCommand
                    |
                    v
               Dynamics
                    |
                    v
            AircraftState update
```

## Responsibility separation

### Mission

Describes **what the vehicle should achieve**.

Examples:

```python
Mission(
    goto=(1000, 500, 300)
)
```

or:

```python
Mission(
    flight_plan=[
        waypoint_1,
        waypoint_2,
        waypoint_3,
    ]
)
```

A mission does not directly control the aircraft.

---

### Autopilot / Guidance

Converts mission objectives into immediate motion targets.

Examples:

For fixed-wing:

```python
MotionCommand(
    target_heading=90,
    target_speed=22,
    target_altitude=300,
)
```

For multirotor:

```python
MotionCommand(
    target_position=(100, 50, 30)
)
```

The autopilot understands the vehicle type and decides the appropriate guidance strategy.

---

### Dynamics

The dynamics model only answers:

> Given the current state and a desired motion command, what is the next state?

The common interface:

```python
def step(
    self,
    state: AircraftState,
    command: MotionCommand,
    perf: Performance,
    dt: float,
) -> AircraftState:
```

The dynamics model does **not** know about:

- missions
- waypoints
- flight plans
- navigation logic
- autopilot behavior

It only applies vehicle constraints.

---

# MotionCommand Interface

A common motion command structure:

```python
MotionCommand(
    target_position=None,
    target_velocity=None,
    target_heading=None,
    target_speed=None,
    target_altitude=None,
    target_vertical_speed=None,
)
```

Not every field needs to be populated.

Different vehicle types interpret the same command differently.

---

# Vehicle Dynamics

## Multirotor

A multirotor is approximately holonomic.

Capabilities:

- hover
- stop
- move laterally
- fly directly to a point
- rotate independently from velocity direction

Example:

```python
MotionCommand(
    target_position=(100, 50, 30)
)
```

The dynamics model:

- computes desired movement direction
- applies acceleration limits
- applies velocity limits
- updates position

Example:

```python
MotionCommand(
    target_velocity=(3, 1, 0)
)
```

means:

> Move with this velocity vector, subject to performance limits.

The dynamics model does not need to simulate:

- roll angle
- pitch angle
- motor thrust
- body rates

unless a lower-level simulator is required.

---

# Fixed-wing

A fixed-wing aircraft is non-holonomic.

It must respect:

- minimum airspeed
- maximum airspeed
- turn rate / turn radius
- climb rate
- descent rate
- acceleration limits

Example:

```python
MotionCommand(
    target_position=(100, 50, 300)
)
```

does not mean:

> Move directly toward this point.

Instead the dynamics model must enforce:

- forward flight
- feasible turning
- altitude changes
- speed constraints

The resulting path is different from a multirotor.

A fixed-wing cannot:

- stop at the waypoint
- move sideways
- instantly change direction

---

Example command:

```python
MotionCommand(
    target_heading=90,
    target_speed=22,
    target_altitude=300,
)
```

is a natural fixed-wing command.

The dynamics model maintains:

- heading convergence
- airspeed
- altitude response

---

# VTOL

VTOL combines both vehicle types.

A VTOL has multiple flight modes:

```python
VTOLMode:
    MULTIROTOR
    TRANSITION
    FIXED_WING
```

In multirotor mode:

```
VTOL Dynamics
      |
      v
Multirotor Model
```

In fixed-wing mode:

```
VTOL Dynamics
      |
      v
Fixed-wing Model
```

The transition model handles:

- mode switching
- changing aerodynamic behavior
- changing performance limits

Low-level transition details should not leak into the command interface.

---

# Example: Same Command, Different Behavior

Both vehicles receive:

```python
MotionCommand(
    target_position=(1000, 500, 300),
    target_speed=20,
)
```

The resulting trajectory depends on the vehicle.

## Multirotor

- flies directly to the point
- can slow down
- can stop and hover

## Fixed-wing

- turns toward the point
- maintains forward speed
- follows a curved feasible trajectory
- may enter loiter after arrival

## VTOL

Depends on current flight mode.

---

# Design Principles

The simulator should follow:

```
Mission
    = What should happen

Autopilot
    = How should we achieve it

Dynamics
    = What is physically possible
```

This separation gives several benefits:

- Different autopilots can control the same aircraft model.
- The same autopilot interface can support multiple vehicles.
- PX4/ArduPilot-like systems can be integrated later.
- Vehicle physics remain isolated and testable.
- New vehicle types can implement the same dynamics interface.

The dynamics layer should remain a vehicle physics model, not a navigation system.

## Autopilot Update Rate and MotionCommand Generation

A mission command is not sent directly to the dynamics model.

For example:

```python
Mission(
    waypoint=(1000, 500, 300)
)
```

is processed by the autopilot/guidance layer, which periodically generates a `MotionCommand`.

The update rates are independent:

```
Mission / Autopilot:     ~1 Hz
Dynamics integration:    ~50 Hz
```

The loop is:

```
Mission
    |
    v
Autopilot.step()
    |
    v
MotionCommand
    |
    v
Dynamics.step()
    |
    v
AircraftState
```

The same `MotionCommand` may be held for multiple dynamics integration steps:

```python
motion_cmd = autopilot.step(state, mission)

for _ in range(n):
    state = dynamics.step(
        state,
        motion_cmd,
        perf,
        dt,
    )
```

This avoids coupling navigation logic to the physics integration rate and better reflects real flight systems.

## Autopilot and SeparationManager Interaction

The autopilot and separation management are separate layers with different responsibilities.

The autopilot answers:

> "How should the aircraft achieve its mission?"

The separation manager answers:

> "Is this command safe given nearby traffic, and should it be temporarily overridden?"

The control flow is:

```
Mission
    |
    v
Autopilot
    |
    v
Nominal MotionCommand
    |
    v
SeparationManager
    |
    v
Final MotionCommand
    |
    v
Dynamics
    |
    v
AircraftState
```

The autopilot continuously produces the aircraft's intended motion:

```python
nominal_command = autopilot.step(
    state,
    mission,
    perf,
)
```

The separation manager receives this command and decides whether to:

1. Allow the nominal command.
2. Replace it with an avoidance command.
3. Return control to the autopilot after recovery.

Example:

Normal flight:

```python
nominal_command = MotionCommand(
    target_heading=90,
    target_speed=20,
)

final_command = nominal_command
```

Conflict detected:

```python
nominal_command = MotionCommand(
    target_heading=90,
    target_speed=20,
)

final_command = MotionCommand(
    target_heading=140,
    target_speed=18,
)
```

After recovery:

```python
final_command = nominal_command
```

The separation manager does not own navigation. It does not know how to fly to a waypoint, follow a route, or maintain a cruise profile. It only temporarily modifies the aircraft's motion when required for safety.

The existing `_decide()` logic maps naturally into this layer:

```
_decide()
    |
    +-- Conflict Detection
    |
    +-- Conflict Resolution
    |
    +-- Recovery Management
```

and should become:

```python
class SeparationManager:

    def step(
        self,
        state: AircraftState,
        perceived_traffic: list[AircraftState],
        nominal_command: MotionCommand,
        perf: Performance,
        dt: float,
    ) -> MotionCommand:
        ...
```

The separation manager may maintain internal pair memory such as:

- active conflicts
- resolution state
- onset velocity
- recovery status

but it should not replace the autopilot.

When recovery criteria are satisfied, the separation manager releases control by returning the autopilot's current nominal command.

## So what should FinalSetpoint contain?

```python
class FixedWingSetpoint:
    heading: float
    airspeed: float
    altitude: float
```

```python
class MultirotorSetpoint:
    velocity: Vector3
    yaw_rate: float | None
```

## High level dataflow

```
Autopilot + SeparationManager
          |
          v
Vehicle-specific velocity/heading setpoint
          |
          v
Dynamics
```
