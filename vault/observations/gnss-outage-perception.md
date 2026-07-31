# A GNSS outage sweep saturates almost immediately — the rate barely matters once it fires

**Status: illustrative.** [[0021-navigation-extension-by-quality-effects|`GnssOutage`]] is the
reference [[0021-navigation-extension-by-quality-effects|`NavEffect`]], and the obvious first
question is whether it moves a safety number at all — the check
[[todo-might-be-a-bug]] §7 says a new model always needs, since a flat sweep is exactly what an
unwired model produces. It does move it, hard. The second result was not the question: **the sweep
is almost entirely a step at "any outage at all", not a gradient in the rate.** Written 2026-07-31.

Reproduce with 250 pairwise encounters per cell, `pos_ci95 = 25 m`, `vel_ci95 = 2 m/s`,
`pos_factor = 10`, latching (`recover_rate = 0`), MVP + Past-CPA, `t_max = 300 s`, seed 11, one
`nav` substream per encounter:

| `fail_rate` [1/h] | mean time to outage [s] | P(LoS) | mean `min_sep` [m] |
|---|---|---|---|
| 0 | ∞ | 0.004 | 186.4 |
| 300 | 12.0 | 0.648 | 45.4 |
| 1200 | 3.0 | 0.672 | 45.0 |
| 4800 | 0.8 | 0.692 | 43.6 |

## The step is between "never" and "ever", not across the rate

P(LoS) goes 0.004 → 0.648 on the first non-zero cell and then moves 0.044 over a **16×** further
increase in rate. `min_sep` does the same thing: 186 m → 45 m, then 1.8 m across the rest.

That is what a *latching* failure in a bounded encounter has to look like. The encounters here run
well past 60 s, so a 12 s mean time to outage already means nearly every encounter degrades at some
point; raising the rate past that only moves *when*, and once an aircraft is flying on a 250 m
95%-radial fix it is in trouble regardless of whether that started at t = 3 s or t = 12 s. The
sweep is therefore measuring `P(outage occurs)`, which has already saturated, and not the rate.

**The practical consequence for an experiment design:** a `fail_rate` axis is close to useless in
this regime. Sweep something that stays informative after the outage has landed — `pos_factor`,
`recover_rate` (which decides how long a degraded aircraft stays degraded), or `declare` — and use
`fail_rate` as a two-level on/off rather than a gradient. The saturation is a property of the
scenario length, so a much shorter encounter or a much lower rate would put the interesting region
back in range.

## Why this pathway is IPS-blind, and precisely which part is

`GnssOutage`'s docstring carries the warning [[0019-channel-extension-by-link-gates|`RadioHealth`]]
carries, for the same reason: a latching outage is a **discrete jump** that `min_sep` carries no
information about, so the splitting shells cannot steer toward it — the pathway measured collapsing
8/8 replications in [[important-ips-gap]].

The distinction worth recording is that this is a property of the *jump*, not of navigation effects
generally, and the N side makes that visible in a way the C side did not:

- A **continuous** accuracy degradation *is* coupled to `min_sep` — a bigger position error gives
  worse geometry gives less separation — so IPS reaches it fine.
- A **permanently** degraded sensor needs no effect at all. It is just a larger `pos_ci95`, which
  is an ordinary continuous axis.
- Only `fail_rate > 0` is unreachable, because only that introduces the jump.

So the IPS-blind set here is exactly `fail_rate > 0`, and the saturation above is the reason that
matters less than it sounds: the rate was not a useful axis anyway. Estimate an outage study by
plain MC, or condition on the failure time and reweight.

## Relations

- Measures [[0021-navigation-extension-by-quality-effects]]'s reference effect; §6 of that ADR
  states the IPS caveat this note quantifies.
- The C-side twin is [[transceiver-outage-perception]] — same hazard law, same latching shape,
  and the same "does it move a number" check.
- The failure mode this note exists to rule out is [[todo-might-be-a-bug]] §7: a sweep that comes
  out flat because nothing reads the parameter.
