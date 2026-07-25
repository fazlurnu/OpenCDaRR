# Imposing DORCA's directional selectivity — where it lives in our code, and why "just replace the MVP bias" is only half of it

**Status: design note, not implemented.** Reading of Niu, Ma & Han, *"Directional optimal
reciprocal collision avoidance"* (DORCA), *Robotics and Autonomous Systems* 136 (2021) 103705
([`vault/papers/dorca.pdf`](../papers/dorca.pdf)), mapped onto our resolvers. Records how the
right-hand-rule constraint would enter `cr/vo.py` and `cr/mvp.py`, and the caveats that stopped us
from doing it now. Written 2026-07-25. **No code changed.** Related:
[[headon-threshold]], [[multi-intruder-vo-vs-mvp]].

## What DORCA actually adds to ORCA

Plain ORCA/VO leaves the velocity obstacle on **whichever cone edge is nearer** the preferred
velocity — an arbitrary, per-step left-or-right choice. That is the non-determinism the paper
targets: it prevents unified right-of-way rules and makes multi-aircraft encounters disorderly
(secondary conflicts when everyone returns to course). DORCA forces a **deterministic side** —
deviate to starboard — by splitting the VO cone at its centerline and keeping only the half that
corresponds to a right-hand deviation (`DVO = VO ∩ {L(v) > 0}`, Eq. 23), then building the permitted
half-plane from that half-cone.

Two clarifications the paper's framing invites:

- **"Turn right" is per-aircraft, relative to each ownship's own heading** — not a shared compass
  direction. Two aircraft on opposite headings both turn to their own right and therefore turn
  *opposite* absolute ways, which is what makes them pass port-to-port. The shared thing is the
  *rule*, not the maneuver.
- **It is reciprocal, not classic right-of-way.** DORCA drops ORCA's responsibility coefficient
  (η → 1, each aircraft plans fully for itself) so **both** aircraft in a pair maneuver. Nobody
  holds course. This differs from ICAO/FAA, where the give-way aircraft turns and the other holds.
  DORCA only faithfully reproduces the **head-on** ICAO case; "both turn right" for crossing and
  overtaking is DORCA's own simplification, not the actual regulation. A real gap if we ever claim
  rule-compliance realism.

## In `cr/vo.py` — a side filter on the candidate set

Our VO ([`opencdarr/cr/vo.py`](../../opencdarr/cr/vo.py)) is the non-directional union-of-cones
resolver: it collects boundary candidates (edge projections + union vertices) and picks the one
nearest the preferred velocity (`min(feasible, ...)`). Directionality is a **filter** on that set —
keep only starboard exits, then nearest-wins among those. The primitive is a 2D cross product in our
ENU (x = East, y = North) frame:

```python
def _is_right_turn(cur_e, cur_n, cand_e, cand_n):
    """cand is a starboard (clockwise) deviation from the current velocity."""
    return cur_e * cand_n - cur_n * cand_e < 0.0   # z of (current × candidate)
```

Applied to the `feasible` list, using `velocity_enu(own)` as the reference (so each aircraft
measures *its own* right). The paper's DVO centerline split is the more robust form — it guarantees
the permitted half is non-empty and references the *relative*-velocity geometry — but the
cross-product filter captures the head-on and most crossing cases. Over-constrained fallback
(`_penetration`) would prefer starboard and relax to port only when starboard is truly infeasible.

## In `cr/mvp.py` — "replace the bias" is real, but only fixes the head-on case

MVP ([`opencdarr/cr/mvp.py`](../../opencdarr/cr/mvp.py)) chooses a side arbitrarily in exactly **one**
place — the bias fallback (line 47):

```python
cx, cy = ry / dist * d_miss, -rx / dist * d_miss  # perpendicular to r: pick a side
```

`(ry, −rx)` is a fixed rotation of the **line-of-sight** vector — not referenced to the ownship
heading, so not guaranteed starboard. Replacing it with an ownship-referenced right-hand normal
(pick the perpendicular sign so `v_own − dv` is a clockwise turn) makes the **head-on** case
DORCA-compliant. That is genuinely worth doing: it is the same case as our [[headon-threshold]]
livelock.

**But the bias block only fires in the near-head-on degenerate case** (`d_miss <= _BIAS_EPS`). For
every other geometry — crossing, converging, overtaking — MVP never reaches line 47; it steers away
on whichever side the natural CPA offset falls, which may be **port**. So replacing the bias gives:

- ✅ head-on symmetry resolved to starboard (and fixes the livelock);
- ❌ **not full DORCA** — crossing/converging still resolve to the natural potential-field side.

Full directionality in MVP means forcing the side for **every** pair (flip `(cx, cy)` to starboard
before computing the gain whenever the natural side is port). And that carries a real caveat: **MVP
is a summed potential field** (`v_own − Σ dv_i`). Forcing each pairwise `dv` to starboard and *then*
summing does not preserve MVP's tangent-to-the-zone guarantee the way DORCA's velocity-space
half-plane **intersection** does — two starboard-forced vectors can under- or over-shoot. A
"directional MVP" is therefore a hybrid, not a faithful DORCA; faithful directionality belongs in
the VO/half-plane resolver.

## Decision

Not implementing now. When we do:

1. **VO** is the right home for faithful DORCA — add the cross-product side filter first, then
   consider the centerline DVO split for robustness.
2. **MVP** bias replacement is a clean, well-scoped fix for the head-on livelock — bill it honestly
   as "MVP with a right-of-way head-on tiebreak", **not** "DORCA in MVP".
