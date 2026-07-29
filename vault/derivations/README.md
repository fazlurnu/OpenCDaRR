# derivations

The math, worked out — dCPA distance, projected-normal, IPS level crossings — in LaTeX,
each linked to the module that implements it and the test that checks it. Duplication of an
equation here that helps a reviewer verify it is worth it (design-philosophy #11).

10 notes so far, covering every shipped algorithm: `conflict-geometry`, `cpa-detection`,
`mvp-resolution`, `ftr-recovery`, `pastcpa-recovery`, `probabilistic-ftr-recovery`, `l1-guidance`,
`gps-noise`, `step-dynamics-m600`, `fixedwing-coordinated-turn`. Like the ADRs they are dated
records: `step-dynamics-m600` derives the M600 kinematics that phase 4 later folded into
`opencdarr/dynamics/multirotor.py`, so a module path in an older note may name a file that has
since moved.
