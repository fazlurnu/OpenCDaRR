# derivations

The math, worked out — dCPA distance, projected-normal, IPS level crossings — in LaTeX,
each linked to the module that implements it and the test that checks it. Duplication of an
equation here that helps a reviewer verify it is worth it (design-philosophy #11).

11 notes so far, covering every shipped algorithm: `conflict-geometry`, `cpa-detection`,
`mvp-resolution`, `dh-orca-resolution`, `ftr-recovery`, `pastcpa-recovery`,
`probabilistic-ftr-recovery`, `l1-guidance`, `gps-noise`, `step-dynamics-m600`,
`fixedwing-coordinated-turn`. `dh-orca-resolution` is the exception: it derives the ORCA
half-plane, the linear program, and DH-ORCA's second constraint, but the implementations were
removed, so it stands as a derivation ahead of the code rather than a record of it.

Like the ADRs they are dated records: `step-dynamics-m600` derives the M600 kinematics that phase
4 later folded into `opencdarr/dynamics/multirotor.py`, so a module path in an older note may name
a file that has since moved.
