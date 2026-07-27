# Unified IPS — one coordinate for nav AND comms rare events (opus4p8)

Minimal encounter (T=60, rpz=1.0, d_target=1.5); nav = continuous drift (sigma_nav=0.1), comms = discrete breach after L_crit=17 dropped broadcasts (rx=0.58). IPS: 12 reps x 4000 particles x 14 shells; MC ground truth over 5,000,000 encounters/regime.

## MC ground truth (Wilson 95% CI)

| regime | P(LoS) | 95% CI | events |
| --- | --- | --- | --- |
| nav | 5.388e-04 | [5.19e-04, 5.60e-04] | 2694 |
| comms | 1.040e-05 | [7.93e-06, 1.36e-05] | 52 |
| both | 5.406e-04 | [5.21e-04, 5.61e-04] | 2703 |

## IPS estimate per (coordinate x regime) — PASS = CI overlaps MC

| coordinate | nav (drift) | comms (jumps) | nav + comms | regimes |
| --- | --- | --- | --- | --- |
| `min_sep` | 5.13e-04 PASS | collapse | 5.11e-04 PASS | **2/3** |
| `staleness` | collapse | 1.06e-05 PASS | 1.04e-05 FAIL | **1/3** |
| `unified` | 5.13e-04 PASS | 1.02e-05 PASS | 5.03e-04 PASS | **3/3** |

`min_sep` ladders the continuous nav drift but reads a **structural zero** on the rare discrete comms pathway (separation is bimodal — nominal or breached — so its intermediate shells hold no partial progress). It still passes *nav+comms* because nav dominates the total — confirming the escape-hatch in `important-ips-gap.md`, while failing the pure comms pathway. `staleness` is the mirror image: it ladders the drop run but is identically zero under perfect comms, and undercounts *nav+comms* by missing the nav contribution entirely.

`unified = max(nav_progress, cap*comm_progress)` reduces to each single coordinate in that coordinate's own regime and ladders whichever pathway a particle is advancing when both are on, so it tracks MC in **all three** regimes. The Blom lesson made concrete: nest the shells on the rare-event driver, and when there are two drivers, take the per-pathway max.
