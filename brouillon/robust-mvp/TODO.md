# robust-mvp — checklist

1. [x] **Derive and implement RMVP** — `rmvp.py`. MVP's rotation with a self-consistent angular
   margin; the chance constraint moves from the offset domain (unbounded, infeasible at shallow
   crossings) to the angle domain (bounded, always feasible).
2. [x] **Verify** — `verify_rmvp.py`, `fig_validation.png`. `k = 0` and `sigma -> 0` both give MVP
   (3.8e-07 relative, pure round-off between the erratum and angle forms — checked identical to 25
   digits in `mpmath`); root residual 9.8e-13 rad over 480 cases; step bounded as `|v_rel| -> 0`;
   delivered `P(d > rpz)` measured end-to-end through `GnssNavigation`.
3. [x] **Probe** — `probe_mc.py`, 400 encounters per cell, zero LoS observed. The ladder rule was
   rewritten after the first pilot collapsed both 30 deg / 1 m/s cells: shells now sit at quantiles
   of the probe's own minimum-separation distribution, not at a geometric descent in excess.
4. [x] **IPS campaign, `vel_ci95` 3 m/s** — `ips_run.py`, 600 particles x 8 reps, 0/8 collapses in
   every cell. RMVP 15x lower P(LoS) at 2 deg and 64x at 30 deg, intervals disjoint in both rows.
5. [x] **VO-frame figures** — `fig_vo_frame.py`. MVP against RMVP under position, velocity, and
   both, at the campaign range and at a 45 deg cone (`--gamma 45`), with the M600 envelope drawn.
   Both write into `vault/derivations/img/`, which is the single copy.
6. [x] **Derivation** — `vault/derivations/robust-mvp-resolution.md`, with all three figures.
   Covers why the angle domain, the "is this only an adaptive margin?" question, every result so
   far, and seven named limitations. UAMVP is compared in the text where it was measured (§5, §7,
   §8.1) but is not plotted: the figures are MVP against RMVP, which is what the campaign ran.
   ← **current**
7. [x] **Exact-quantile variant** — `rmvp_exact.py`, the projected-normal quantile in place of the
   Gaussian one. Calibrates where the confidence is attainable (30 deg: 0.988 -> 0.957 delivered,
   with a 20% *smaller* step) and changes almost nothing at 2 deg (0.848 -> 0.855). IPS at 2 deg,
   (10 m, 3 m/s): 6.083e-05 [3.79e-05, 7.52e-05] against the Gaussian's 7.614e-05
   [5.86e-05, 9.09e-05] — intervals overlap.
8. [x] **Teaching notebook** — `rmvp_explained.ipynb`, nine steps, MVP shown at each one.
   Executed end to end with `nbclient`; every number in it is computed live. ← **current**
9. [x] **Derivation corrected** — §9.1 and §9.6 rewritten, §8.5 and §8.6 added. ← **current**
10. [ ] **README** — the folder's own front page, pointing at the derivation rather than repeating
   it.
11. [ ] **IPS campaign, `vel_ci95` 1 m/s** — the other half of the original grid, deprioritised in
   favour of 3 m/s. Needs a pilot first: the old ladder collapsed both 30 deg cells, and the new
   quantile ladder has not been piloted at 1 m/s.

## Corrections made along the way

- **§9.1 blamed the wrong thing.** It attributed RMVP's 2 deg shortfall to the Gaussian angular
  approximation. Swapping in the exact projected-normal quantile moves that cell by 0.007, so the
  approximation was not what was binding. The real cause is §9.6: 0.95 is unattainable by any
  rotation in 63.5% of perceived geometries there (median ceiling 0.876, delivered 0.855). §9.6 in
  turn described itself as a corner case needing `|v_rel|` below 2.3e-3 m/s, which describes only
  when the rotation cap binds arithmetically, not how often the target is out of reach. Both are
  rewritten, and §9.1 keeps the retraction visible rather than quietly replacing the text.
- The claim that removing an artificial `pi/2 - 1e-3` rotation clamp "collapsed the step from 2.62
  to 0.10 m/s" was **wrong**. Measured, the clamp truncates the step by a uniform ~13% and only in
  the cap branch, which needs `|v_rel|` below 2.3e-3 m/s at the 2 deg geometry and never fires in
  the campaign. The physical cap `pi/2 - alpha` is still the right one, because it is the one with
  a geometric meaning — not because the alternative was measurably wrong.
