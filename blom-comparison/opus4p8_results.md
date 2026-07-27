# Blom car example — IPS reproduction (opus4p8)

Fixed-effort multilevel splitting, N_p=10000, m=10 shells, dt=0.01 s, 20 replications. Exact p_hit = exp(-60/mu) (paper Table 4).

*Exponential sampling* draws the reaction time in one shot; with no Brownian motion clones then never diverge, so it collapses for mu <= 5 (paper Table 5). *Bernoulli sampling* rolls the reaction per step so clones diverge (paper Table 6).

### Paper's equidistant position shells (D_k, Section 5.3)

| mu (s) | exact p_hit | exponential sampling | Bernoulli sampling |
| --- | --- | --- | --- |
| 10 | 2.479e-03 | 2.229e-03  95%CI[1.75e-03, 2.47e-03] | 2.453e-03  95%CI[2.27e-03, 2.60e-03] |
| 5 | 6.144e-06 | collapsed | 5.920e-06  95%CI[0.00e+00, 1.65e-05]  [3/20 reps collapsed] |
| 3.33 | 1.523e-08 | collapsed | 3.377e-08  95%CI[0.00e+00, 6.75e-07]  [19/20 reps collapsed] |
| 2.5 | 3.775e-11 | collapsed | collapsed |
| 2 | 9.358e-14 | collapsed | collapsed |

Position shells reproduce Tables 5/6: exponential collapses for mu <= 5; Bernoulli reaches mu = 5 but starves in the deep tail, because a braked car coasts ~1800 m so the first shells select nobody and the still-reacting sub-population is spent before selection begins. (The paper's own Table 6 shows a larger-than-the-estimate error at mu = 2, the same edge.)

### Delay-progress shells (nested on the rare-event driver)

| mu (s) | exact p_hit | exponential sampling | Bernoulli sampling |
| --- | --- | --- | --- |
| 10 | 2.479e-03 | 2.406e-03  95%CI[2.00e-03, 2.62e-03] | 2.488e-03  95%CI[2.45e-03, 2.53e-03] |
| 5 | 6.144e-06 | 1.548e-05  95%CI[0.00e+00, 1.57e-04]  [17/20 reps collapsed] | 6.216e-06  95%CI[6.07e-06, 6.35e-06] |
| 3.33 | 1.523e-08 | collapsed | 1.522e-08  95%CI[1.46e-08, 1.57e-08] |
| 2.5 | 3.775e-11 | collapsed | 3.768e-11  95%CI[3.59e-11, 3.92e-11] |
| 2 | 9.358e-14 | collapsed | 9.211e-14  95%CI[8.51e-14, 9.74e-14] |

Nesting the shells on how far the car coasts while still reacting gives uniform per-shell survival, so Bernoulli reaches the full mu = 2 tail (~1e-13) accurately. Exponential still collapses under the identical shells — isolating the *sampling* scheme as the paper's point, while the importance function decides how deep Bernoulli can go.
