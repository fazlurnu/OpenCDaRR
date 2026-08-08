# The OpenCDaRR curriculum

Runnable notebooks, one for each lesson of [`docs/curriculum.md`](../../docs/curriculum.md).

The curriculum document holds the plan: the levels, the goals, and the check questions. This folder
holds the code. Read the lesson in the document, then do the notebook.

Every notebook obeys ASD-STE100 Simplified Technical English. Every notebook uses the public API
only. No notebook needs `scripts/`.

## Setup

Install the library from your clone, one level above this folder:

```bash
pip install -e ".[examples]"
```

Then open a notebook and run it from the top to the bottom:

```bash
jupyter lab
```

## The two paths

Each lesson is **core** or **depth**. Core lessons give the breadth. Depth lessons give the detail.

| Path | Lessons | Time |
| --- | --- | --- |
| Short | The core lessons only | About 12 hours |
| Full | All lessons | About 40 hours |

Do the short path first. Then come back for the depth lessons that your work needs.

## Level 0 — Setup

| Notebook | Type | Time | Goal |
| --- | --- | --- | --- |
| [`L0_1_install_and_test.ipynb`](L0_1_install_and_test.ipynb) | core | 30 min | You have an environment that runs the library and its tests. |
| [`L0_2_the_first_result.ipynb`](L0_2_the_first_result.ipynb) | core | 45 min | You know the shape of an answer before you know how to make one. |
| [`L0_3_public_surface.ipynb`](L0_3_public_surface.ipynb) | core | 20 min | You know what the library gives you, and where each name lives. |

## Level 1 — The parts, one at a time

Level 1 uses **no simulation loop**. Every lesson calls one module directly, so you see what each
one does before anything hides it. Lesson L1.10 builds a small loop by hand from the parts, which is
the bridge into Level 2.

| Notebook | Type | Time | Goal |
| --- | --- | --- | --- |
| [`L1_1_aircraft_state.ipynb`](L1_1_aircraft_state.ipynb) | core | 30 min | Make one aircraft and read its state. |
| [`L1_2_frame_and_geometry.ipynb`](L1_2_frame_and_geometry.ipynb) | depth | 40 min | Calculate a bearing, a distance, and a relative state. |
| [`L1_3_performance_envelope.ipynb`](L1_3_performance_envelope.ipynb) | core | 30 min | Describe an airframe as data. |
| [`L1_4_motion_command.ipynb`](L1_4_motion_command.ipynb) | core | 40 min | Write the command that all the modules speak. |
| [`L1_5_kinematics_multirotor.ipynb`](L1_5_kinematics_multirotor.ipynb) | core | 45 min | Move one multirotor. |
| [`L1_6_kinematics_fixedwing.ipynb`](L1_6_kinematics_fixedwing.ipynb) | core | 60 min | Move one fixed-wing, and know why it is different. |
| [`L1_7_autopilot_and_mission.ipynb`](L1_7_autopilot_and_mission.ipynb) | core | 40 min | Give an aircraft a plan; one autopilot serves both airframes. |
| [`L1_8_conflict_detection.ipynb`](L1_8_conflict_detection.ipynb) | core | 40 min | Predict a conflict. |
| [`L1_9_conflict_resolution.ipynb`](L1_9_conflict_resolution.ipynb) | core | 60 min | Calculate the way out of one. |
| [`L1_10_recovery.ipynb`](L1_10_recovery.ipynb) | core | 45 min | Decide when to go back to the plan. |
| [`L1_11_wind.ipynb`](L1_11_wind.ipynb) | core | 40 min | Add wind, and know which speed each module uses. |
| [`L1_12_cns_navigation.ipynb`](L1_12_cns_navigation.ipynb) | core | 60 min | Make an aircraft measure itself with an error. |
| [`L1_13_navigation_degradation.ipynb`](L1_13_navigation_degradation.ipynb) | depth | 40 min | Make an error that continues across steps. |
| [`L1_14_cns_communication.ipynb`](L1_14_cns_communication.ipynb) | core | 60 min | Delay, lose, and space out the messages. |
| [`L1_15_link_gates.ipynb`](L1_15_link_gates.ipynb) | depth | 40 min | Turn a directed link off for a physical reason. |
| [`L1_16_cns_surveillance.ipynb`](L1_16_cns_surveillance.ipynb) | core | 30 min | Know what the separation logic reads. |
| [`L1_17_rng_and_repeatability.ipynb`](L1_17_rng_and_repeatability.ipynb) | core | 40 min | Make your results repeat. |

## Level 2 to Level 8

In preparation. See [`docs/curriculum.md`](../../docs/curriculum.md) for the plan:

| Level | Subject |
| --- | --- |
| 2 | One pairwise simulation — the parts put together. |
| 3 | Many encounters — from one run to a rate. |
| 4 | Comparison — more than one module at the same time. |
| 5 | More than two aircraft — rings, traffic, mixed fleets. |
| 6 | Rare events — where counting stops. |
| 7 | Write your own module. |
| 8 | Run a full experiment. |

## How these notebooks differ from `examples/handbook/`

The handbook notebooks are the source of the pages on
[opencdarr.github.io](https://opencdarr.github.io). They are reference material, and each one covers
one module in full.

These notebooks are a course. They have an order, they build on each other, and each one ends with
a check question. Use the handbook notebook for the same module when you want more detail.
