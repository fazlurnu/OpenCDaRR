# The rare-event validation ladder: a noise knob from P(LoS)=1 down to 5×10⁻⁴

**Status: measured (Phase 8 prep, pre-IPS).** The calibration target for the Blom–Bakker IPS gate
([[phase-8-plan]]): a regime where brute-force MC measures a *non-zero* loss-of-separation rate, so
"IPS agrees with MC in a not-too-rare regime" (how-to Step 6) has something to agree *with*. Every
CDR cell in the handbook MC showed **P(LoS) = 0** — the resolver is simply too good at the noise
levels we run — so before IPS there was no MC anchor at all. This note finds the knob that turns the
rare event on continuously, measures the ladder, and pins the two rungs the IPS validation will use.
Written 2026-07-26.

**The knob is GNSS self-noise.** Sweep the ownship/intruder declared accuracy `pos_ci95` (with
`vel_ci95 = pos_ci95/10`) with everything else fixed: a `dcpa=0` crossing pair (`sample_pairwise`,
`speed 10.2889 m/s`, `tlos 90 s`), `rpz 50 m`, `lookahead 120 s`, `StateBased` detection,
`MVP(margin=1.05)` resolution, `PastCPA(bouncing_guard=True)` recovery, `GnssNavigation` (default
Gaussian), `dt 0.5 s`, `t_max 250 s`. Each point is `estimate_ipr` over N seeded encounters;
`P(LoS) = n_los/n_conflict` with a Wilson 95% CI. (Probe run over the plain-MC path,
`opencdarr/estimator.py`; the sweep driver is a scratch script that should be promoted to
`scripts/ips_validation_probe.py` for one-command reproduction — a loose end.)

## The ladder

| `pos_ci95` [m] | N | P(LoS) | 95% CI | regime |
|---|---|---|---|---|
| resolver off | 400 | 1.0000 | [0.990, 1.000] | trivial — mechanism sanity |
| 80 | 1000 | 0.1870 | [0.164, 0.212] | common |
| 60 | 1000 | 0.1030 | [0.086, 0.123] | common |
| **40** | 1000×3 | **~0.028** | pooled 83/3000 | **IPS validation anchor** |
| 30 | 1000 | 0.0090 | [0.005, 0.017] | getting rare |
| 20 | 1000 | 0.0050 | [0.002, 0.012] | MC/IPS crossover |
| **10** | 10000×3 | **4.7×10⁻⁴** | [2.8e-4, 7.8e-4] | **rare — IPS demonstration** |

`P(LoS)` is monotone in `pos_ci95` across three orders of magnitude — a clean, continuous rarity dial.

## The headline: at pos=10 a single 10⁴-sample MC run reads exactly zero

Three seeds at `pos_ci95 = 10`, N = 10000 each:

| seed | LoS events | P(LoS) | 95% CI |
|---|---|---|---|
| 0 | **0** / 10000 | 0.00000 | [0, 3.8e-4] |
| 1 | 6 / 10000 | 0.00060 | [2.8e-4, 1.3e-3] |
| 2 | 8 / 10000 | 0.00080 | [4.1e-4, 1.6e-3] |
| pooled | 14 / 30000 | **4.7×10⁻⁴** | [2.8e-4, 7.8e-4] |

Seed 0 found **no events in ten thousand encounters** — a single MC run there reports P(LoS)=0, i.e.
"no risk," which is wrong. It took all 30 000 pooled to get 14 events and a CI that finally excludes
zero. This is the rare-regime failure of Monte Carlo in one table: at N = 10⁴ the estimator is not
merely imprecise, it can land on the wrong answer. To get even ~10 % relative precision on
P ≈ 4.7×10⁻⁴ needs ~100 events → **~200 000 encounters (~85 min serial here)**; 5 % needs ~800 000
(~5–6 h) — for *one* cell. That cost, growing as the event rarefies, is exactly what IPS exists to
cut.

## Why `pos_ci95` works as a rarity knob

The resolver clears *by design*: MVP targets `margin·rpz` and reaches it, so the deterministic
clearance is roughly noise-independent, always just above the boundary ([[fleet-ipr-sweep]] — "the
margin is pinned to rpz, so noise-robustness is what thins"). What tips a *cleared* pair into LoS is
therefore not a worse mean margin but the **perceived-position error**: bigger `pos_ci95` fattens the
tail of each broadcast fix, so more often an aircraft resolves against a wrong-enough picture to
under-clear, and the pair rides through the boundary. LoS is the tail of that error distribution, and
the knob widens the distribution — hence the smooth, monotone climb. It is the pairwise, single-cell
face of the density-driven erosion in [[fleet-ipr-sweep]] and the stale-picture erosion in
[[communication-reception-latency]]: same mechanism (safety pinned to `rpz`, noise doing the tipping),
different axis.

## What this anchors for IPS ([[phase-8-plan]])

- **`pos_ci95 = 40` (P ≈ 0.028) is the validation anchor.** Plain MC gives a solid CI here in a few
  thousand encounters, so the Step-6 gate — *IPS must agree with brute-force MC* — is testable.
- **`pos_ci95 = 10` (P ≈ 4.7×10⁻⁴) is the demonstration target.** Walk the knob down 40 → 20 → 10 and
  watch MC's CI blow up (and a single run read 0) while IPS holds. That divergence *is* the payoff
  figure.
- **The gate, concretely:** IPS ≈ MC at pos=40, then IPS stays tight and consistent down to pos=10.
  Only trust IPS in the genuinely rare regime once it has passed at pos=40.

## What this doesn't cover (and the honest limits)

- **LoS is not collision.** The rare event here is `min_sep < rpz` (50 m), not a physical collision
  (`min_sep <` a few m). The true target event is orders of magnitude rarer, where MC is outright
  infeasible — that is the real IPS regime, and this ladder only reaches its doorstep.
- **The anchor needs more than N = 1000.** At pos=40 the single-seed estimates ranged 0.016–0.038
  (16/29/38 events per 1000); the pooled 0.028 is stable but a production anchor wants N ≈ several
  thousand for a tight CI. The pos=10 point is already pooled to 30 000 for the same reason.
- **One geometry, one method.** A `dcpa=0` crossing pair, N = 2, `MVP + PastCPA` only. Other
  resolvers ([[multi-intruder-vo-vs-mvp]] — VO provisional), recovery criteria, crossing angles
  ([[near-parallel-ipr-inversion]]), wind ([[ipr-under-wind]]), or the lossy-comm axis
  ([[fleet-lossy-ipr]]) would each shift the ladder. The *shape* — a monotone noise dial spanning
  1 → 5×10⁻⁴ — is the transferable result; the exact rungs are for this cell.
- **Reproducibility loose end.** The sweep script is currently scratch; promote it to
  `scripts/ips_validation_probe.py` (seeded, `--pos`/`--n` args) so this table regenerates from one
  command, like the other observations.
