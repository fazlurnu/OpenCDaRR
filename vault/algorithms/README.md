# algorithms

**Reserved, never populated.** This directory holds nothing but this file.

The intent was one note per algorithm — detection, resolution, recovery, estimators — each
pointing to the module that implements it. In practice that material went to two places instead,
and splitting it a third way was not worth it:

- **The math** lives in [`../derivations/`](../derivations/), one note per equation set, each
  linked to its module and its test — `cpa-detection`, `mvp-resolution`, `ftr-recovery`,
  `pastcpa-recovery`, `probabilistic-ftr-recovery`, and the rest.
- **The behaviour** lives in [`../observations/`](../observations/), one note per finding, each
  linked to the experiment that shows it.

Look there first. If a genuinely algorithm-level note is ever needed — one that is neither a
derivation nor a finding — this is where it goes.
