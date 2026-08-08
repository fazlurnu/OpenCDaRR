# OpenCDaRR curriculum

A course that teaches you to use OpenCDaRR. It starts with one part of the library. It ends with a
full experiment that you can publish.

This text obeys ASD-STE100 Simplified Technical English. The sentences are short. The words are
simple. This is a teaching document, not a reference. The reference is the handbook at
[opencdarr.github.io](https://opencdarr.github.io) and the docstrings in the code.

---

## 1. Before you start

### 1.1 Who this course is for

You must know these things before you start:

- Python 3.11, at the level of a function, a class, and a dataclass.
- NumPy arrays, at the level of an array and a random generator.
- The basic idea of air traffic separation: two aircraft must stay apart.

You do not need these things:

- Experience with BlueSky or with an other traffic simulator.
- Knowledge of rare-event statistics. Level 6 teaches it.
- Knowledge of flight dynamics. Level 1 teaches the two models that the library has.

### 1.2 What you can do at the end

After the full course you can do these tasks:

1. Build one encounter between two aircraft, and read its result.
2. Add measurement noise, a datalink, and wind to that encounter.
3. Measure a loss-of-separation probability from many encounters.
4. Compare two conflict resolvers on the same encounter set.
5. Write your own detector, resolver, recovery rule, airframe, or noise shape.
6. Measure a probability that is too small for plain counting.
7. Run a full experiment, and record the data that makes it repeatable.

### 1.3 The two paths

Each lesson has a type. **Core** lessons give the breadth. **Depth** lessons give the detail. Do
the core lessons first.

| Path | Lessons | Time | Result |
| --- | --- | --- | --- |
| Short | The core lessons only | About 12 hours | You can run and compare experiments. |
| Full | All lessons | About 40 hours | You can extend the library and defend your numbers. |

Do the short path first. Then come back for the depth lessons that your work needs.

### 1.4 The shape of each lesson

Each lesson has these parts:

- **Goal** — what you can do at the end of the lesson.
- **Read** — the code or the notebook to read first.
- **Do** — the steps. Write the code yourself. Do not copy a notebook.
- **Check** — a question that you must answer. If you cannot answer it, do the lesson again.

Write your work in a notebook or a script of your own. Keep it. Level 8 uses it.

### 1.5 The mental model

Keep this picture in your mind for the whole course. One simulation step has this order:

```
mission -> autopilot -> [CNS: navigation -> communication -> surveillance] ->
detection -> resolution -> recovery -> kinematics -> new state
```

Two rules control everything in this library:

1. **The separation logic sees the perceived state, not the true state.** The CNS layer makes the
   perceived state. The measurement of the result uses the true state.
2. **Each module is a class with one method.** To change the behaviour, you give a different
   object. You do not edit the loop.

---

## Level 0 — Setup

### L0.1 Install and test — Core — 30 min

**Notebook.** [`examples/curriculum/L0_1_install_and_test.ipynb`](../examples/curriculum/L0_1_install_and_test.ipynb).

**Goal.** You have a Python environment that runs the library and its tests.

**Do.**

1. Make the environment and install the library with the examples extra:

```bash
conda create -n opencdarr python=3.11 && conda activate opencdarr && pip install -e ".[examples]"
```

2. Run the test suite:

```bash
pytest -q
```

3. Import the library and print the version:

```python
import opencdarr
print(opencdarr.__version__)
```

**Check.** The test suite is green. What does the `examples` extra add that the core install does
not have?

### L0.2 The first result — Core — 45 min

**Notebook.** [`examples/curriculum/L0_2_the_first_result.ipynb`](../examples/curriculum/L0_2_the_first_result.ipynb).

**Goal.** You know the shape of an answer before you know how to make one.

**Do.**

1. Run one encounter. It gives one result: a conflict flag, a loss flag, and a minimum separation.
2. Add a sensor. Run the same geometry with 8 seeds. Each seed gives a different answer.
3. Run 2000 encounters with `estimate_p_los`. Read the rate and put an interval on it.
4. Sweep the position accuracy with `run_experiment`. Plot P(LoS) and the median closest approach.

**Check.** The median minimum separation stays flat while the probability of a loss increases by a
large factor. Say in one sentence why the median cannot show the risk.

Note: `examples/handbook/tutorial_your_first_experiment.ipynb` covers the same ground but calls
`estimate_ipr` and `wilson_interval`, which the library no longer has. Use the curriculum notebook
until that one is updated.

### L0.3 Read the public surface — Core — 20 min

**Notebook.** [`examples/curriculum/L0_3_public_surface.ipynb`](../examples/curriculum/L0_3_public_surface.ipynb).

**Goal.** You know what the library gives you, and where each name lives.

**Read.** The docstring of `opencdarr/__init__.py`, and its `__all__` list.

**Do.** Sort each name in `__all__` into one of four groups:

- A **contribution surface** — an abstract class that you can subclass.
- A **reference model** — a class that you can use immediately.
- A **runner** — a function that flies aircraft or counts results.
- A **value** — plain data that you construct.

**Check.** `MVP` is in which group? `ConflictResolver` is in which group? Why do both exist?

---

## Level 1 — The parts, one at a time

This level uses no simulation loop. You call each module directly. This is the slow part of the
course. It is also the part that makes Level 2 easy.

Lesson L1.10 is the one exception, and it is deliberate: it builds a small loop by hand from the
parts of L1.5 to L1.9, because a recovery rule cannot be shown at one instant. That hand-built loop
is the bridge into `run_fleet` at L2.1.

### L1.1 The aircraft state — Core — 30 min

**Notebook.** [`examples/curriculum/L1_1_aircraft_state.ipynb`](../examples/curriculum/L1_1_aircraft_state.ipynb).

**Goal.** You can make one aircraft and read its state.

**Read.** `opencdarr/state.py`.

**Do.**

1. Make an aircraft with `create_aircraft(M600, id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)`.
2. Print each field. Find `trk`, `gs`, `yaw`, `bank`, `flight_time`, and `distance_flown`.
3. Set `pos_ci95=10.0`. Read the docstring. Find what the field declares.

**Check.** `trk` is the direction of travel. `yaw` is the direction of the nose. Give an example
of an aircraft where the two are different.

### L1.2 The frame and the geometry — Depth — 40 min

**Notebook.** [`examples/curriculum/L1_2_frame_and_geometry.ipynb`](../examples/curriculum/L1_2_frame_and_geometry.ipynb).

**Goal.** You can calculate a bearing, a distance, and a relative state.

**Read.** `opencdarr/geo.py` and `opencdarr/relative.py`.

**Do.**

1. Use `geo.forward()` to move a point 1000 m to the north. Check the new latitude.
2. Use `geo.qdrdist()` to get the bearing and the distance back to the first point.
3. Make two aircraft. Use `relative_enu()` to get the relative position and velocity.
4. Use `segment_min_range()` on two relative states from two steps. Compare the result with the
   distance at each of the two steps.

**Check.** Why can the minimum separation in one step be smaller than the distance at the start of
the step and at the end of the step?

### L1.3 The performance envelope — Core — 30 min

**Notebook.** [`examples/curriculum/L1_3_performance_envelope.ipynb`](../examples/curriculum/L1_3_performance_envelope.ipynb).

**Goal.** You can describe an airframe as data.

**Read.** `opencdarr/performance.py`. Look at `M600` and `SMALL_FIXEDWING`.

**Do.**

1. Print the fields of `M600` and of `SMALL_FIXEDWING`. Find the fields that differ.
2. Make your own `Performance` for a small racing multirotor. Give it a high `v_max` and a high
   `ax`.
3. Give your multirotor a `phi_max` of 0. Give a fixed-wing a `phi_max` of 35.

**Check.** A multirotor has `yaw_rate_max` but a fixed-wing does not use it. Why does a fixed-wing
turn rate come from `phi_max` and the speed?

### L1.4 The motion command — Core — 40 min

**Notebook.** [`examples/curriculum/L1_4_motion_command.ipynb`](../examples/curriculum/L1_4_motion_command.ipynb).

**Goal.** You can write the command that all the modules speak.

**Read.** The docstring of `MotionCommand` in `opencdarr/kinematics/base.py`.

**Do.**

1. Make a command with `MotionCommand.from_track_speed(90.0, 10.0)`. Print `v_east` and `v_north`.
2. Make the same command with `MotionCommand.from_velocity(10.0, 0.0)`. Compare the two.
3. Make a position command with `target_position=(52.01, 4.0)`.
4. Make a fixed-wing command with `target_course` and `target_airspeed`.

**Check.** Each field defaults to `None`. Why is `None` different from 0 for `target_yaw`?

### L1.5 The multirotor kinematics — Core — 45 min

**Notebook.** [`examples/curriculum/L1_5_kinematics_multirotor.ipynb`](../examples/curriculum/L1_5_kinematics_multirotor.ipynb).

**Goal.** You can move one multirotor for one step.

**Read.** `opencdarr/kinematics/multirotor.py` and
`examples/handbook/kinematics_multirotor.ipynb`.

**Do.**

1. Make a `Multirotor()` and an `M600` state.
2. Call `step()` with a velocity command for 30 steps of 1 s. Record the track.
3. Command a speed that is higher than `v_max`. Show that the model limits it.
4. Command a large speed change. Show that `ax` makes a ramp, not a jump.
5. Use `target_yaw` to point the nose to the east while the aircraft moves to the north.

**Check.** The multirotor is a holonomic point mass. What does that permit that a fixed-wing
cannot do?

### L1.6 The fixed-wing kinematics — Core — 60 min

**Notebook.** [`examples/curriculum/L1_6_kinematics_fixedwing.ipynb`](../examples/curriculum/L1_6_kinematics_fixedwing.ipynb).

**Goal.** You can move one fixed-wing for one step, and you know why it is different.

**Read.** `opencdarr/kinematics/fixedwing.py`, `examples/handbook/kinematics_fixedwing.ipynb`,
and `docs/fixedwing-vs-bluesky.md`.

**Do.**

1. Give a fixed-wing a velocity command that points 180° from its track. Record the path.
2. Record the bank angle at each step. Find the effect of `roll_rate_max`.
3. Increase the airspeed. Show that the turn radius increases.
4. Give a `target_position` and a `target_leg_start`. Show that the aircraft flies the leg line,
   not a direct line to the point.

**Check.** The fixed-wing cannot stop. What does it do at its last waypoint?

### L1.7 The autopilot and the mission — Core — 40 min

**Notebook.** [`examples/curriculum/L1_7_autopilot_and_mission.ipynb`](../examples/curriculum/L1_7_autopilot_and_mission.ipynb).

**Goal.** You can give an aircraft a plan, and let one autopilot serve both airframes.

**Read.** `opencdarr/mission.py`, `opencdarr/autopilot/`, and `examples/handbook/autopilot.ipynb`.

**Do.**

1. Make a `Mission(goto=(52.02, 4.01))`.
2. Make a `WaypointAutopilot(mission)`. Call `step()` and print the command.
3. Give the same autopilot to a multirotor and to a fixed-wing. Compare the two paths.
4. Make a `Mission` with a `flight_plan` of three waypoints. Change `capture_radius`. Find the
   effect on the corner.

**Check.** The autopilot gives a position setpoint, not a velocity. Why does this make one
autopilot enough for two very different airframes?

### L1.8 Conflict detection — Core — 40 min

**Notebook.** [`examples/curriculum/L1_8_conflict_detection.ipynb`](../examples/curriculum/L1_8_conflict_detection.ipynb).

**Goal.** You can predict a conflict between two aircraft.

**Read.** `opencdarr/cd/base.py` and `opencdarr/cd/statebased.py`.

**Do.**

1. Make two aircraft that converge. Call `StateBased().detect()` with `rpz=50` and
   `t_lookahead=120`.
2. Move the aircraft apart until the detector says no. Record the distance.
3. Decrease `t_lookahead` to 30 s. Find the new distance.
4. Use `is_los()` to test the true loss of separation. Compare it with the detection.

**Check.** Detection is a prediction. A loss of separation is a fact. Name one condition where the
detector says yes but no loss happens.

### L1.9 Conflict resolution — Core — 60 min

**Notebook.** [`examples/curriculum/L1_9_conflict_resolution.ipynb`](../examples/curriculum/L1_9_conflict_resolution.ipynb).

**Goal.** You can calculate an avoidance command from a conflict.

**Read.** `opencdarr/cr/base.py`, `opencdarr/cr/mvp.py`, and `opencdarr/cr/vo.py`.

**Do.**

1. Make one conflict. Call `MVP(1.05).resolve()`. Print the new velocity.
2. Change the margin to 1.5. Compare the size of the change in velocity.
3. Call `VO().resolve()` on the same conflict. Compare the two commands.
4. Plot the two commands as vectors with the original velocity.

**Check.** MVP moves the aircraft to the edge of the protected zone at the closest point. VO
selects a velocity that is not in a cone. Say one condition where the two give a different answer.

Note: the VO results in this library are provisional. The implementation is our own.

### L1.10 Recovery — Core — 45 min

**Notebook.** [`examples/curriculum/L1_10_recovery.ipynb`](../examples/curriculum/L1_10_recovery.ipynb).

**Goal.** You can decide when an aircraft can go back to its plan.

**Read.** `opencdarr/crr/base.py`, `pastcpa.py`, `ftr.py`, and `probabilistic_ftr.py`.

**Do.**

1. Fly one conflict with `MVP` but with no recovery. Show that the aircraft never returns.
2. Add `PastCPA()`. Find the time when it resumes.
3. Add `FTR()` instead. Compare the resume time and the achieved separation.
4. Add `ProbabilisticFTR()`. Read its docstring. Find the parameter that sets the confidence.

**Check.** `PastCPA` waits until the aircraft are past the closest point. `FTR` resumes as soon as
the direct path is clear. Which one gives more separation, and which one gives a shorter flight?

### L1.11 Wind — Core — 40 min

**Notebook.** [`examples/curriculum/L1_11_wind.ipynb`](../examples/curriculum/L1_11_wind.ipynb).

**Goal.** You can add wind, and you know which speed each module uses.

**Read.** `opencdarr/wind.py` and the wind functions in `opencdarr/relative.py`.

**Do.**

1. Make a wind with `WindField.from_met(270.0, 8.0)`. Print `components()`, `speed()`, and
   `coming_from()`.
2. Fly a fixed-wing to the north in this wind. Record the ground track and the heading.
3. Calculate the crab angle with `wind_correction_angle()`. Compare it with the recorded value.
4. Fly the same leg with `NO_WIND`. Compare the flight times.

**Check.** `from_met()` takes the direction that the wind comes from. The `components()` point
downwind. Why do the two conventions both exist?

### L1.12 CNS: navigation — Core — 60 min

**Notebook.** [`examples/curriculum/L1_12_cns_navigation.ipynb`](../examples/curriculum/L1_12_cns_navigation.ipynb).

**Goal.** You can make an aircraft measure itself with an error.

**Read.** `opencdarr/cns/navigation.py`, `opencdarr/cns/noise_distributions.py`, and
`examples/handbook/navigation.ipynb`.

**Do.**

1. Make a `GnssNavigation()`. Call `measure()` 1000 times on the same true state.
2. Plot the measured positions. Check that about 95 % are in the `pos_ci95` radius.
3. Change the shape to `make_mixture_gaussian()`. Plot again. Find the tail.
4. Change the shape to `make_anisotropic_gaussian()`. Find the direction of the larger error.
5. Set `pos_ci95_declared` to a value that is not the true accuracy. Read what the aircraft
   broadcasts.

**Check.** All four shapes hold the same 95 % containment. Why is this necessary for a fair
comparison between them?

### L1.13 CNS: navigation degradation — Depth — 40 min

**Notebook.** [`examples/curriculum/L1_13_navigation_degradation.ipynb`](../examples/curriculum/L1_13_navigation_degradation.ipynb).

**Goal.** You can make an error that continues across steps.

**Read.** `GnssOutage` and `NavEffect` in `opencdarr/cns/navigation.py` and
`opencdarr/cns/base.py`.

**Do.**

1. Add a `GnssOutage(fail_rate=..., recover_rate=...)` to the navigation model.
2. Record the `NavQuality` of one aircraft at each step. Find the periods of the outage.
3. Set `declare=False`. Find what changes in the broadcast.

**Check.** A per-step noise draw has no memory. An outage has memory. Why does an outage need a
state and a noise draw does not?

### L1.14 CNS: communication — Core — 60 min

**Notebook.** [`examples/curriculum/L1_14_cns_communication.ipynb`](../examples/curriculum/L1_14_cns_communication.ipynb).

**Goal.** You can delay, lose, and space out the messages between aircraft.

**Read.** `opencdarr/cns/communication.py`, `opencdarr/cns/broadcast.py`, and
`examples/handbook/communication.ipynb`.

**Do.**

1. Make a `Comm(reception_prob=0.9)`. Count the delivered messages.
2. Add a latency with `constant_latency()`, then `uniform_latency()`, then
   `lognormal_latency()`. Compare the delivery times.
3. Make a `BroadcastSchedule(interval=2.0)`. Find the effect on the update interval.
4. Add `jitter`. Remember that jitter needs its own random generator.
5. Add a random phase. Show that the aircraft no longer transmit together.
6. Give `reception_prob` a mapping of one directed link, such as `{("OWN", "INT"): 0.5}`. Show
   that the opposite link is not affected.

**Check.** A reception probability of 0.9 does not give an update interval of 1.11 s. Say why the
mean interval is longer than that.

### L1.15 CNS: link gates — Depth — 40 min

**Notebook.** [`examples/curriculum/L1_15_link_gates.ipynb`](../examples/curriculum/L1_15_link_gates.ipynb).

**Goal.** You can turn a directed link off for a reason that is physical.

**Read.** `RadioHealth`, `SurveillanceRange`, and `TransceiverComm` in
`opencdarr/cns/communication.py`.

**Do.**

1. Use a `TransceiverComm` with a `RadioHealth` gate. Break one transmitter. Show that the other
   aircraft still transmits.
2. Add a `SurveillanceRange(max_range=...)`. Show first contact at the range boundary.

**Check.** A link is directed. Aircraft A can see B while B cannot see A. Name one real system
where this happens.

### L1.16 CNS: surveillance — Core — 30 min

**Notebook.** [`examples/curriculum/L1_16_cns_surveillance.ipynb`](../examples/curriculum/L1_16_cns_surveillance.ipynb).

**Goal.** You know what the separation logic reads.

**Read.** `opencdarr/cns/surveillance.py` and `opencdarr/cns/stack.py`.

**Do.**

1. Make a `LastKnown()`. Read the perceived traffic at each step of a short run.
2. Use `age()` to get the age of each message. Plot the age against the time.
3. Remove the communication model. Find what the perceived traffic becomes.

**Check.** Before the first message, the perceived state of a neighbour is absent. Why is absent
different from a state that is old?

### L1.17 Random numbers and repeatability — Core — 40 min

**Notebook.** [`examples/curriculum/L1_17_rng_and_repeatability.ipynb`](../examples/curriculum/L1_17_rng_and_repeatability.ipynb).

**Goal.** Your results are the same every time you run them.

**Read.** `opencdarr/rng.py`.

**Do.**

1. Make a root sequence with `root_seed_sequence(42)`.
2. Spawn 4 children with `spawn()`. Make a generator from each with `generator()`.
3. Run the same encounter twice with the same seed. Compare the results field by field.
4. Give the navigation noise and the communication loss the same generator. Explain the problem.

**Check.** Each source of randomness gets its own substream. Why does one shared generator break
the comparison between two conditions?

---

## Level 2 — One pairwise simulation

Now you connect the parts. Every lesson in this level uses `run_fleet`.

### L2.1 The agent and the runner — Core — 45 min

**Goal.** You can fly two aircraft to the end of an encounter.

**Read.** `Agent`, `Airframe`, and `run_fleet` in `opencdarr/fleet.py`.

**Do.**

1. Make two `AircraftState` objects that converge.
2. Put each one in an `Agent(state, M600)`.
3. Call `run_fleet(agents, rpz=50, t_lookahead=120, dt=1.0, detector=StateBased())`.
4. Print `conflict`, `los`, `min_sep`, `n_los_pairs`, and `n_los_aircraft`.
5. Add `resolver=MVP(1.05)`. Compare `min_sep` with the first result.
6. Add `recovery=PastCPA()`. Compare again.

**Check.** With no resolver, the run has a loss of separation. With a resolver, it does not. What
does the recovery rule change, if the loss does not occur in both?

### L2.2 The termination rules — Depth — 30 min

**Goal.** You know when a run stops, and why.

**Read.** `FleetEnv.is_terminal` in `opencdarr/fleet.py`.

**Do.**

1. Run one encounter. Record the number of steps.
2. Change `done_timeout` from 10 s to 60 s. Compare the number of steps.
3. Set `t_max` to a small value. Show that the run stops early.
4. Give each aircraft a mission and set `stop_within=50`. Compare the stop time.

**Check.** A fixed-wing orbits its last waypoint at its loiter radius. What is the smallest
`stop_within` that lets a fixed-wing register as complete?

### L2.3 Make a conflict on purpose — Core — 40 min

**Goal.** You can build a geometry that gives a conflict at a known time.

**Read.** `create_conflict` in `opencdarr/scenario/pairwise.py`.

**Do.**

1. Make an ownship. Call `create_conflict(own, intr_id="INT", dpsi=90, dcpa=0, tlos=60, rpz=50)`.
2. Fly the pair with no resolver. Check that the loss happens at about `tlos`.
3. Change `dcpa` to 60 m. Check that no loss occurs.
4. Change `side` from 1 to -1. Compare the two geometries.
5. Change `dpsi` from 90 to 20, then to 160. Compare the results.

**Check.** `dcpa` is the miss distance with no resolution. Why does a `dcpa` of 60 m give no loss
when `rpz` is 50 m?

### L2.4 Record and plot — Core — 40 min

**Goal.** You can see what the numbers describe.

**Read.** `opencdarr/viz.py`.

**Do.**

1. Run one encounter with `record=True`.
2. Call `plot_pairwise(run, rpz=50)`. Look at the tracks and the separation history.
3. Call `extract_tracks(run)`. Plot the tracks yourself with matplotlib.
4. Plot the separation against the time. Draw a line at `rpz`.

**Check.** `record=True` does not change the trajectory. Why not, and why is the option not the
default?

### L2.5 Add navigation noise — Core — 45 min

**Goal.** You can make the aircraft act on a state that is not true.

**Do.**

1. Take the run of L2.3 with `MVP` and `PastCPA`. It has no loss.
2. Add `navigation=GnssNavigation()` with `pos_ci95=10.0`, and give it an `rng`.
3. Run it 200 times with 200 different seeds. Count the losses.
4. Increase `pos_ci95` to 40 m. Count again.
5. Plot the 200 minimum separations as a histogram for both levels.

**Check.** The same geometry now gives different results. Which state does the detector read, and
which state does `min_sep` measure?

### L2.6 Change the noise shape — Depth — 30 min

**Goal.** You know that the size of the error is not the whole story.

**Do.**

1. Repeat L2.5 with `make_mixture_gaussian()` at the same `pos_ci95`.
2. Compare the median minimum separation. Compare the count of the losses.
3. Repeat with `make_anisotropic_gaussian()`.

**Check.** The three shapes have the same 95 % radius. Why can the loss counts be different?

### L2.7 Add the datalink — Core — 45 min

**Goal.** You can make the two aircraft see each other differently.

**Do.**

1. Add `communication=Comm(...)` and `surveillance=LastKnown()` to the run of L2.5.
2. Give the communication its own `comm_rng`.
3. Decrease the reception probability. Count the losses.
4. Increase the broadcast interval to 4 s. Count the losses.
5. Set `share_intent=True`. Compare the results.

**Check.** This is asymmetric situational awareness. Explain it in two sentences with your own
plot.

### L2.8 Add wind — Core — 30 min

**Goal.** You can add an environment that moves the whole encounter.

**Do.**

1. Add `wind=WindField.from_met(270.0, 10.0)` to the run of L2.7.
2. Compare `min_sep` and the loss count with the run with no wind.
3. Increase the wind speed until the fixed-wing cannot hold its track. Record the speed.

**Check.** Wind moves both aircraft. Why does it still change the separation between them?

### L2.9 Add a mission — Depth — 40 min

**Goal.** Your aircraft go somewhere, and they go back to the plan after they avoid.

**Do.**

1. Give each aircraft a `WaypointAutopilot` with a mission.
2. Run with `MVP` and `PastCPA`. Look at the return to the leg.
3. Set `stop_within` to end the run at the last waypoint.
4. Compare the flight time and the distance flown with and without the conflict.

**Check.** This gives you an efficiency metric as well as a safety metric. Name the two fields of
`AircraftState` that hold it.

---

## Level 3 — Many encounters

One run is an anecdote. This level makes it a measurement.

### L3.1 From one run to a rate — Core — 45 min

**Goal.** You can estimate a probability with `estimate_p_los`.

**Read.** `opencdarr/estimate/montecarlo.py` and `examples/handbook/monte_carlo.ipynb`.

**Do.**

1. Make an encounter builder with `pairwise(M600)`.
2. Load a `Config` from `configs/pairwise.yaml`, or make one in Python.
3. Call `estimate_p_los(build, config, StateBased(), MVP(1.05), PastCPA(), GnssNavigation())`.
4. Print `p_los_run`, `median_min_sep`, `n_los`, and `n_encounters`.
5. Run with 200, 2000, and 20000 encounters. Compare the three estimates.

**Check.** The estimate moves when you increase the sample. When can you stop? Give a rule that
uses the count of the events, not the count of the runs.

### L3.2 The sampled encounter — Core — 40 min

**Goal.** You can control the distribution of the geometry, not one geometry.

**Read.** `sample_pairwise` and `Draw` in `opencdarr/scenario/`.

**Do.**

1. Sample 500 encounters with `sample_pairwise`. Plot the distribution of `dpsi` and of `dcpa`.
2. Pin `dpsi=90`. Sample again. Compare the distributions.
3. Give `dpsi` a custom `Draw` that returns a normal distribution about 45°.
4. Give `gs_intr` a `Draw`. Show that a mixed-speed encounter set works.

**Check.** A pinned value and a custom draw are different tools. Which one belongs in a sweep, and
which one belongs in the encounter model?

### L3.3 The metrics — Core — 45 min

**Goal.** You count a loss of separation in the correct way for your fleet size.

**Read.** `examples/handbook/p_los_metrics.ipynb` and the properties of `MonteCarloEstimate`.

**Do.**

1. Run a pairwise study. Compare `p_los_run` and `p_los_ac`. They are equal.
2. Run a three-aircraft study where only two lose separation. Compare the two metrics again.
3. Read `mean_k`, `sum_a`, and `sum_n`. Say what each one counts.
4. Read `detection_rate`. Say why it is a diagnostic and not a result.

**Check.** In dense traffic, `p_los_run` goes to 1. Why does this make it useless to compare two
resolvers?

### L3.4 The measurement area — Depth — 40 min

**Goal.** You measure only in the region where the traffic is homogeneous.

**Read.** `opencdarr/measurement.py` and `opencdarr/scenario/random_traffic.py`.

**Do.**

1. Make a `Disc(centre, radius)`. Test `contains()` and `area()`.
2. Pass `area=` to `estimate_p_los`. Compare the result with the result with no area.
3. Use `measurement_area()` and `aircraft_for_density()` to size a traffic scenario.

**Check.** The spawn area is larger than the measurement area. Say why the boundary of the spawn
area would give a wrong density.

### L3.5 Combine and split — Depth — 25 min

**Goal.** You add two estimates without a mistake.

**Read.** `combine_p_los` in `opencdarr/estimate/montecarlo.py`.

**Do.**

1. Run two batches of 1000 encounters with different seeds.
2. Combine them with `combine_p_los`. Compare with one batch of 2000.
3. Try to combine them by a mean of the two rates. Find the error.

**Check.** Why must you add the counts and not the rates?

---

## Level 4 — Compare more than one module at the same time

This is the level that turns the library into a research tool.

### L4.1 The declaration — Core — 60 min

**Goal.** You declare an experiment once, and get one row for each condition.

**Read.** The docstring of `opencdarr/experiment/__init__.py`.

**Do.**

1. Make a `Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA())`.
2. Call `run_experiment({"pos_ci95": Fixed(10.0)}, methods=..., backend=MC(2000), base_config=...)`.
3. Print `res.records()` and `res.frame()`.
4. Change `Fixed(10.0)` to `Sweep([0.0, 5.0, 10.0, 20.0, 40.0])`. Print the table.
5. Plot the result with `res.plot("p_los_run")`.

**Check.** `Fixed` and `Sweep` are the only two roles. Where does a random distribution belong, if
it does not belong here?

### L4.2 What you can sweep — Core — 40 min

**Goal.** You know every axis that the declaration accepts.

**Read.** `opencdarr/experiment/declaration.py` and
`examples/handbook/what_you_can_sweep.ipynb`.

**Do.** Make one small sweep for each of these groups:

| Group | Example keys |
| --- | --- |
| Scenario | `speed`, `dcpa_max`, `tlos`, `pos_ci95`, `vel_ci95`, `pos_ci95_declared` |
| Conflict | `rpz`, `t_lookahead` |
| Simulation | `dt`, `t_max`, `done_timeout`, `broadcast_interval`, `broadcast_jitter`, `stop_within` |
| Geometry | `dpsi`, `dcpa`, `side`, `gs_intr` |
| Component | `detector`, `resolver`, `recovery`, `navigation`, `communication`, `surveillance`, `kinematics`, `perf`, `wind`, `airframes`, `scenario` |

**Check.** An unknown key fails immediately. Why is this better than a keyword that the code
ignores?

### L4.3 Compare two resolvers — Core — 45 min

**Goal.** You can put two algorithms in one table.

**Read.** `examples/handbook/resolver_comparison.ipynb`.

**Do.**

1. Declare `{"resolver": Sweep([MVP(1.05), VO()])}`.
2. Add a second axis: `{"pos_ci95": Sweep([5.0, 10.0, 20.0, 40.0])}`.
3. Run the 8 conditions. Print the table.
4. Report two metrics: `p_los_run` and `median_min_sep`.
5. Plot the two response curves on one figure.

**Check.** One resolver can have fewer losses and less separation at the same time. Say why you
must report both metrics.

### L4.4 Compare a whole stack — Core — 60 min

**Goal.** You compare more than one module at the same time, without a mistake.

**Do.**

1. Declare three axes: the resolver, the recovery rule, and the position accuracy.
2. Run the full grid. Count the conditions before you run it.
3. Find the condition with the lowest `p_los_run`.
4. Hold the resolver constant. Plot the recovery rule against the accuracy.

**Check.** A grid of 3 axes grows fast. Give a rule to decide which axis to make a sweep and which
axis to hold fixed.

### L4.5 Read the result — Core — 30 min

**Goal.** You can get to the raw data behind any cell of the table.

**Read.** `opencdarr/experiment/results.py`.

**Do.**

1. Call `res.cell(resolver=..., pos_ci95=10.0)`. Print the raw estimate.
2. Get the `min_seps` tuple from that cell. Plot its histogram.
3. Call `res.frame()`. Do a group operation with pandas.

**Check.** The table gives you a rate. The cell gives you a sample. Name one question that only
the sample can answer.

### L4.6 Cost, cache, and provenance — Core — 45 min

**Goal.** Your experiment is fast, and someone else can repeat it.

**Read.** `opencdarr/cache.py`, `opencdarr/experiment/card.py`, and
`opencdarr/estimate/parallel.py`.

**Do.**

1. Run a sweep with `n_jobs=1`. Record the time.
2. Run the same sweep with `n_jobs=-1`. Compare the time and the numbers. The numbers must be
   identical.
3. Run it again with `cache=True`. Compare the time.
4. Change one line of the library code. Run again. Show that the cache does not serve the old
   result.
5. Set `card_dir=`. Read the card that the run writes.

**Check.** The cache key includes a fingerprint of the source code. Say what breaks without it.

---

## Level 5 — More than two aircraft

### L5.1 The ring — Core — 45 min

**Goal.** You can build a scenario where every aircraft conflicts at the same time.

**Read.** `opencdarr/scenario/ring.py` and `examples/handbook/circle_scenario.ipynb`.

**Do.**

1. Build a `swap_ring(n=4, radius=1500)`. Fly it with `MVP` and `PastCPA`. Plot the tracks.
2. Increase `n` to 8, then 16, then 24. Record `p_los_ac` for each.
3. Compare `crossing_ring` and `converging_ring` with `swap_ring`.
4. Repeat the worst case with `dt=0.25`. Compare the result with `dt=1.0`.

**Check.** A failure at `dt=1.0` can be an artifact of the step size. Give the test that separates
a true failure from an artifact.

### L5.2 Traffic density — Depth — 60 min

**Goal.** You build traffic instead of one hand-made encounter.

**Read.** `opencdarr/scenario/random_traffic.py` and `examples/handbook/traffic_density.ipynb`.

**Do.**

1. Use `aircraft_for_density()` to get the aircraft count for 10 aircraft/km².
2. Build the scenario with `random_traffic()`. Plot the initial positions and headings.
3. Sweep the density from 5 to 25. Plot `p_los_ac` against the density.
4. Repeat with the full CNS stack. Compare the two curves.

**Check.** The entry-bearing rule uses `arcsin`. Say what a naive perimeter rule does wrong.

### L5.3 The mixed fleet — Core — 45 min

**Goal.** You can fly a multirotor and a fixed-wing in the same encounter.

**Read.** `Airframe` in `opencdarr/fleet.py` and `examples/handbook/mixed_fleet.ipynb`.

**Do.**

1. Make `Airframe(M600, Multirotor())` and `Airframe(SMALL_FIXEDWING, FixedWing())`.
2. Give them to `Methods(airframes=[...])`. Run one encounter.
3. Give the fixed-wing envelope to the multirotor integrator. Read the error.
4. Spawn the fixed-wing below its stall speed. Read the error.

**Check.** The library refuses two mistakes at the line where you write them. Say what each
mistake would do if the library flew it.

---

## Level 6 — Rare events

The target level of safety is about 10⁻⁹ for each flight hour. Counting cannot reach it.

### L6.1 Where counting stops — Core — 40 min

**Goal.** You know when plain Monte Carlo has no answer.

**Do.**

1. Run a sweep that decreases `pos_ci95` step by step. Record `n_los` at each step.
2. Continue until `n_los` is 0. Note the `p_los_run` that the estimator reports.
3. Calculate the number of runs that you need for 50 events at that rate.

**Check.** A report of 0 is not a report of "rare". Say the difference in one sentence.

### L6.2 The splitting estimator — Core — 60 min

**Goal.** You can run an IPS estimate and read its result.

**Read.** `opencdarr/estimate/ips.py` and `examples/handbook/rare_event_ips.ipynb`.

**Do.**

1. Change `backend=MC(20000)` to `backend=IPS(shells=[...], n_particles=2000, reps=8)`.
2. Change nothing else. Run it. Print `p_los_run`, `n_collapsed`, and `n_lineages`.
3. Increase `reps`. Compare the spread between the replications.

**Check.** The declaration did not change when the backend changed. Say why this is the design
that makes the comparison in L6.4 possible.

### L6.3 The shell ladder — Depth — 60 min

**Goal.** You can select the shells, and you know when a ladder fails.

**Read.** `examples/handbook/ips_ladder.ipynb`.

**Do.**

1. Run a small MC pilot. Record the distribution of `min_sep`.
2. Build a ladder from that pilot: from the usual end of an encounter down to `rpz`.
3. Run IPS with the ladder. Record the survival fraction at each level.
4. Build a bad ladder where the last step is much larger than the others. Run it. Find the level
   where the cloud dies.

**Check.** The quantity that decides a ladder is not the probability. What is it?

### L6.4 The agreement test — Core — 60 min

**Goal.** You can defend an IPS number.

**Read.** `examples/handbook/rare_event_mc_vs_ips.ipynb` and `scripts/validation/campaign.py`.

**Do.**

1. Select an **anchor** condition where MC finds more than 50 events.
2. Run MC and IPS on that condition. Compare the two estimates.
3. Select a **target** condition where MC finds no events. Run IPS only.
4. Write the two rows in one table, with the wall time of each backend.

**Check.** Why is the target number believable only because of the anchor row?

### L6.5 Parallel — Depth — 30 min

**Goal.** Your rare-event run uses the whole machine.

**Read.** `opencdarr/estimate/parallel.py`.

**Do.**

1. Run an IPS estimate with `n_jobs=1` and then with `n_jobs=-1`. Compare the numbers field by
   field. They must be identical.
2. Set `reps` to less than the count of the workers. Read how the code shards the particles.

**Check.** The result does not change with the count of the workers. Which design decision gives
this?

---

## Level 7 — Write your own

Each lesson here adds a file. No lesson here edits the simulation loop.

### L7.1 Your own resolver — Core — 60 min

**Goal.** You can put your algorithm against the reference algorithms.

**Read.** `examples/handbook/byo_cdarr.ipynb` and
`examples/02_build_your_own_separation_manager.ipynb`.

**Do.**

1. Subclass `ConflictResolver`. Write `resolve()`. Return a `MotionCommand`.
2. Start with a simple rule: turn 30° to the right for each conflict.
3. Run it in `run_fleet`. Plot the tracks.
4. Put it in a `Sweep` with `MVP` and `VO`. Compare the three in one table.

**Check.** Your resolver receives the perceived states, not the true states. Where does the loop
give it the perceived states?

### L7.2 Your own detector and recovery — Depth — 45 min

**Goal.** You can replace the other two parts of the CDaRR stack.

**Do.**

1. Subclass `ConflictDetector`. Write a detector that uses a simple distance threshold.
2. Subclass `RecoveryCriterion`. Write a rule that waits a fixed time.
3. Run the full stack of your own three modules. Compare with the reference stack.

**Check.** Detection and resolution are separate objects. Name one benefit of the separation.

### L7.3 Your own airframe — Core — 45 min

**Goal.** You can add an aircraft type.

**Read.** `examples/03_build_your_own_performance.ipynb`.

**Do.**

1. Write a `Performance` for a heavy-lift multirotor.
2. Fly it against a `M600` in one encounter. Compare the avoidance.
3. Write a `Performance` with a wrong pair of fields. Read the error that the library gives.

**Check.** Why is a new airframe a value and not a subclass?

### L7.4 Your own kinematics — Depth — 60 min

**Goal.** You can add a vehicle that the library does not model.

**Read.** `opencdarr/kinematics/base.py` and `examples/handbook/byo_full_stack.ipynb`.

**Do.**

1. Subclass `Kinematics`. Write `step()` and `validate_performance()`.
2. Model a simple vehicle: a point mass with a first-order speed lag.
3. Make `validate_performance()` refuse an envelope that your model cannot use.
4. Fly it in a mixed fleet with the reference models.

**Check.** Your `step()` reads only the fields of `MotionCommand` that your vehicle knows. What
must it do when a field that it needs is absent?

### L7.5 Your own uncertainty — Depth — 60 min

**Goal.** You can model a sensor error that the library does not have.

**Read.** `NoiseDistribution` and `NavEffect` in `opencdarr/cns/base.py`.

**Do.**

1. Write a `NoiseDistribution` function. Keep the 95 % containment guarantee.
2. Test the containment with 10000 draws before you use it.
3. Write a `NavEffect` with its own state, such as a slow drift.
4. Sweep your shape against the four built-in shapes.

**Check.** Your distribution must obey the containment guarantee. Say what a comparison means if
it does not.

### L7.6 Your own scenario — Depth — 45 min

**Goal.** You can add an encounter family.

**Read.** `opencdarr/scenario/base.py` and `opencdarr/scenario/README.md`.

**Do.**

1. Subclass `Scenario`. Write `draw()`. Return a fleet.
2. Add `size()` and `measurement_area()`.
3. Add `supports_splitting()`. Read what it controls in the IPS backend.
4. Put your scenario in a `Sweep` with `PairwiseEncounter()`.

**Check.** Your `draw()` takes a generator. Why must it not use any other source of randomness?

---

## Level 8 — Run a full experiment

This level makes one result that another person can repeat.

### L8.1 The question — Core — 30 min

**Goal.** Your experiment has one question that a number can answer.

**Do.**

1. Write your question in one sentence. Example: "Does FTR give more separation than PastCPA when
   the position accuracy is worse than 20 m?"
2. Write the metric that answers it. Write the axes that you must sweep.
3. Write the conditions that you hold fixed, and say why you hold each one.

**Check.** A question that no number can answer is not an experiment. Test your sentence against
this rule.

### L8.2 The design — Core — 45 min

**Goal.** Your sample size and your backend come from a pilot run, not from a guess.

**Do.**

1. Run a small pilot: 200 encounters at the hardest condition and at the easiest condition.
2. Read `n_los` at each. Select MC where the events are many. Select IPS where they are few.
3. Calculate the encounter count that gives at least 50 events for each MC condition.
4. Record the estimated wall time. Compare it with the time that you have.

**Check.** Why do you size the sample from the count of the events and not from the count of the
runs?

### L8.3 The config file — Depth — 30 min

**Goal.** Your run is committable and diffable.

**Read.** `configs/pairwise.yaml`, `opencdarr/config.py`, and `run_one_experiment`.

**Do.**

1. Copy `configs/pairwise.yaml`. Change it for your experiment.
2. Run it with `run_one_experiment(load_config(path))`.
3. Compare the result with the same experiment in Python.

**Check.** A config file names its components as strings. Say the benefit and the limit of this.

### L8.4 The campaign — Core — 90 min

**Goal.** You run the full grid, and you can stop it and start it again.

**Read.** `scripts/validation/campaign.py` and `scripts/validation/run_campaign.py`.

**Do.**

1. Write a script that runs your conditions one at a time.
2. Store each row as JSON as soon as it completes. Include the seed, the wall time, and the
   scenario description.
3. Skip a row that is already stored. Do not record a new time for a cached row.
4. Set `card_dir`. Keep the provenance cards.
5. Run the campaign. Read the log.

**Check.** A cached row keeps its original time. Say why a new time for a cached row would be a
wrong number.

### L8.5 The report — Core — 60 min

**Goal.** Your numbers come with their noise, and their limits.

**Do.**

1. Make one table: one row for each condition, with the metric, the event count, and the time.
2. Give each rate an interval. Do not report a rate that rests on fewer than 5 events without a
   note.
3. Make one figure for the main result. No grid. No figure title. Put the detail in the caption.
4. Write the limits: what your encounter model does not include, and where your numbers stop.

**Check.** Read your own report as a reviewer. Find the one number that a reviewer will attack
first. Add the evidence for it.

---

## Appendix A — The quick map

| You want to... | Use |
| --- | --- |
| Move one aircraft | `Multirotor`, `FixedWing`, `MotionCommand` |
| Describe an airframe | `Performance`, `Airframe` |
| Give a plan | `Mission`, `WaypointAutopilot` |
| Detect, resolve, recover | `StateBased`, `MVP` / `VO`, `PastCPA` / `FTR` / `ProbabilisticFTR` |
| Add measurement error | `GnssNavigation`, the `make_*` noise shapes, `GnssOutage` |
| Add a datalink | `Comm`, `TransceiverComm`, `BroadcastSchedule`, `LastKnown` |
| Add an environment | `WindField` |
| Fly one encounter | `Agent`, `run_fleet` |
| Build a geometry | `create_conflict`, `sample_pairwise`, `pairwise` |
| Build a fleet | `swap_ring`, `crossing_ring`, `converging_ring`, `random_traffic` |
| Measure a rate | `estimate_p_los`, `MonteCarloEstimate` |
| Measure a rare rate | `estimate_rare_prob`, `IPS` |
| Compare conditions | `run_experiment`, `Methods`, `Fixed`, `Sweep`, `MC`, `IPS` |
| Read a result | `ExperimentResult.records`, `.frame`, `.cell`, `.plot` |
| Keep it repeatable | `root_seed_sequence`, `spawn`, `generator`, `cache`, `card_dir` |
| See it | `run_fleet(record=True)`, `plot_pairwise`, `extract_tracks` |

## Appendix B — The notebook map

Read the notebook after you try the lesson. Do not read it first.

| Level | Notebook |
| --- | --- |
| 0 | `handbook/tutorial_your_first_experiment.ipynb`, `handbook/a_first_run.ipynb` |
| 1 | `handbook/kinematics_multirotor.ipynb`, `handbook/kinematics_fixedwing.ipynb`, `handbook/autopilot.ipynb`, `handbook/navigation.ipynb`, `handbook/communication.ipynb`, `handbook/separation.ipynb` |
| 2 | `01_pairwise_conflict.ipynb`, `01_pairwise_conflict_extended.ipynb`, `handbook/the_whole_chain.ipynb` |
| 3 | `handbook/monte_carlo.ipynb`, `handbook/p_los_metrics.ipynb` |
| 4 | `handbook/what_you_can_sweep.ipynb`, `handbook/resolver_comparison.ipynb` |
| 5 | `handbook/circle_scenario.ipynb`, `handbook/traffic_density.ipynb`, `handbook/mixed_fleet.ipynb` |
| 6 | `handbook/rare_event_ips.ipynb`, `handbook/rare_event_ips_illustrated.ipynb`, `handbook/ips_ladder.ipynb`, `handbook/rare_event_mc_vs_ips.ipynb`, `handbook/ring_mc_vs_ips.ipynb` |
| 7 | `02_build_your_own_separation_manager.ipynb`, `03_build_your_own_performance.ipynb`, `handbook/byo_cdarr.ipynb`, `handbook/byo_full_stack.ipynb`, `handbook/build-your-own-distilled.ipynb` |
| 8 | `handbook/validation_campaign.ipynb` |

## Appendix C — The self-test

Answer these 12 questions with no help. If you can answer all of them, you completed the course.

1. Name the modules of one simulation step, in their order.
2. Which state does the resolver read? Which state does `min_sep` measure?
3. Give one geometry where the detector predicts a conflict but no loss occurs.
4. Say the difference between `p_los_run` and `p_los_ac`, and when they are equal.
5. Two noise shapes have the same `pos_ci95`. Say why the loss counts can differ.
6. Say why a reception probability of 0.9 does not give a 1.11 s update interval.
7. Say why each source of randomness needs its own substream.
8. `MVP` and `VO` disagree at a shallow crossing angle. Say why.
9. Say the quantity that decides where the IPS shells go.
10. Say why an IPS number needs an MC anchor.
11. Say why `Sweep` and a custom `Draw` are not the same tool.
12. Your `p_los_run` is 0 after 2000 encounters. Say what you report, and what you do next.

---

## Notes for the author of the course

- The course does not yet look at the site repository. Check this text against the site's
  `STYLE.md` before it becomes an outward-facing page.
- Lesson L1.9 marks the VO results as provisional. Keep this note in each place where VO appears
  in a comparison.
- Each Level 1 lesson must have a runnable cell that needs no simulation loop. Some of these cells
  do not exist yet in the notebooks.
- Level 8 sends the reader to `scripts/validation/`. Those scripts are research code, not shipped
  code. Say this where the level starts, if the level becomes a public page.
