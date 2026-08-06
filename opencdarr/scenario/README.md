# Scenarios

A scenario makes one encounter from one seed. It is the only part of an experiment that changes
between a crossing-angle study, a ring study and a traffic study. The rules, the two estimators, the
cache and the report stay the same.

To add a new experiment family, write a new `Scenario`. You do not change anything else.

![The shipped geometries](../../docs/img/scenario-gallery.png)

*Each panel shows the start position (dot) and the route (arrow) of every aircraft. The routes are
cut to keep the panels readable; the aircraft fly further than the arrows show.*

## The two kinds

Scenarios are **placed** or **drawn**.

A placed scenario has one geometry. It ignores its generator, thus the same seed and a different
seed give the same fleet. The rings and the two-aircraft geometries are placed.

A drawn scenario samples a geometry. It uses its generator for everything, thus a different seed
gives different traffic at the same density. `RandomTraffic` is drawn.

Both give the same answer to `draw(rng, config)`. This is why an estimator does not need to know
which kind it has.

## What is available

| Scenario | Aircraft | Kind | Use it for |
|---|---|---|---|
| `PairwiseEncounter` | 2 | drawn | the encounter distribution, or one pinned geometry |
| `swap_pair` | 2 | placed | a head-on swap |
| `near_parallel` | 2 | placed | a shallow crossing, which closes slowly |
| `SwapRing` | n | placed | `n/2` head-on pairs at the same time |
| `CrossingRing` | n | placed | the same, but through the centre at every `n` |
| `ConvergingRing` | n | placed | a superconflict: all aircraft go to one point |
| `RandomTraffic` | n | drawn | traffic at a given density |

Each class has a function of the same idea below it (`swap_ring`, `crossing_ring`,
`random_traffic`). Use the function to make a fleet directly. Use the class to give an experiment
to `run_experiment`.

## Two rings that are not the same ring

`swap_ring` sends each aircraft to the **start of another aircraft** — the one `n // 2` places
around the ring. `crossing_ring` sends each aircraft to the **opposite point** on the ring.

At an even `n` the two are the same fleet. At an odd `n` they are not.

![swap_ring against crossing_ring](../../docs/img/scenario-ring-variants.png)

At `n = 3` the `swap_ring` routes make a triangle. At `n = 5` they make a star. The routes stay
750 m and 464 m away from the centre. The `crossing_ring` routes always go through the centre.

**This is important for a sweep of the fleet size.** If you step `n` through 3, 4 and 5 with
`swap_ring`, you change the geometry as well as the number of aircraft. A trend in the result then
has two causes, and the result does not tell you which one. Use `crossing_ring`, because only the
number of aircraft changes.

## The release ring and the measurement area

`RandomTraffic` uses two circles. The aircraft start on the outer one and fly inward. Results are
counted only inside the smaller one.

The space between the two circles is flown but not counted, and this is what makes the design work.
At the start, all the aircraft are on the ring. A circle is one-dimensional, thus some of them are
near each other immediately — at 10 aircraft/km² this occurs in every sample. Those aircraft were
never separated, thus to count them is to measure the release rule and not the safety of the
airspace. They are all on the outer circle, which is outside the measured disc, thus the gate keeps
all of them out. An aircraft is counted only after it flies across the space between the circles,
and by then it has flown with the others for some seconds.

If you put the aircraft directly into the measured disc, this protection is lost: two of them can
start nearer than `rpz` with no history, and no area can tell that from a true loss of separation.

**Density is a transient.** `run_fleet` flies a fixed list of aircraft and cannot make new ones
during a run. The paper releases aircraft continuously, which keeps the density constant. Here one
group crosses together: the disc fills, then empties. If your study depends on a constant density,
say which time window you read it over.

The scenario gives the area, because the two are one design. If you had to declare the area
somewhere else, the two declarations could disagree.

```python
traffic = RandomTraffic(density=10.0, radius=1000.0)
traffic.size()              # 31 aircraft
traffic.measurement_area()  # Disc(radius=833 m) -- the paper's 1.35 / 1.62
```

## Speed

Every fleet builder takes one speed for the fleet, or one speed for each aircraft:

```python
swap_ring(4, speed=10.0)                      # all four at 10 m/s
swap_pair(speed=[10.0, 14.0])                 # a multirotor and a fixed-wing
```

A mixed fleet needs the second form. `SMALL_FIXEDWING` has a stall speed of 12 m/s, which is more
than the 10 m/s of a multirotor. Thus one speed for the fleet stops the fixed-wing or flies the
multirotor too fast. `Agent` refuses a speed that is outside the envelope of its airframe.

## Write your own

Make a subclass of `Scenario` and write `draw`:

```python
@dataclass(frozen=True)
class MyEncounter(Scenario):
    n: int = 4

    def draw(self, rng, config):
        # return [(AircraftState, goal_or_None), ...]
        # read speed and the accuracies from `config`, thus a sweep of them reaches the geometry
        ...

    def size(self) -> int:
        return self.n
```

Keep the scenario **independent of the airframe**. Give the states and the goals, and let the
`perf`, `kinematics` or `airframes` of the caller decide what flies them. One geometry can then be
flown by multirotors, by fixed-wings or by a mixed fleet.

A goal of `None` tells the aircraft to hold its cruise. Use it when the encounter has no
destination. A pairwise encounter is an example: the geometry is the experiment, and a destination
would add a manoeuvre that the experiment did not ask for.

Three more methods are available. You do not have to write them.

| Method | Default | Write it when |
|---|---|---|
| `measurement_area()` | `None` (measure everywhere) | the study area is smaller than the flown area |
| `size()` | `None` (not known) | the fleet size is fixed |
| `supports_splitting()` | `True` | the rare-event estimator must not be used |

Give `supports_splitting()` as `False` for a traffic **stream** that continues for hours. In that
case "one loss or more" is not a rare event, thus the smallest separation does not tell the
particles apart and the rare-event estimator gives a number near 1 after a long calculation.

## Figures

The two figures come from `scripts/scenario_gallery.py`:

```bash
PYTHONPATH=. python scripts/scenario_gallery.py
```
