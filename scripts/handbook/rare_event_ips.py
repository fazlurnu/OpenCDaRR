"""Handbook figure: what multi-level splitting does to the sample.

One picture for the "Rare-event simulation" page, from a real :mod:`opencdarr.ips` run on the
handbook's 90 deg crossing encounter:

  1. *left* — the population shell by shell. Each dot is one particle's running-minimum separation
     at the end of its leg, against the shell staircase. After every shell the whole population is
     resampled back onto the far side of it, so a fixed budget of particles keeps being re-spent on
     the trajectories that are still closing.
  2. *right* — the same run as a product. Survival fractions that are individually unremarkable
     multiply down to the rare probability, which is the entire trick.

The loop below mirrors :func:`opencdarr.ips.ips_once` step for step using only public functions
(``evolve_shard`` + ``resample_level``), so it can record each particle's achieved running-minimum;
the estimate is asserted equal to ``ips_once`` on the same seed before anything is drawn.

Handbook plot style: no suptitle, concise titles, no grid. Writes into the site repo.

    PYTHONPATH=. python scripts/handbook/rare_event_ips.py
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from opencdarr.cd import StateBased  # noqa: E402
from opencdarr.cns.navigation import GnssNavigation  # noqa: E402
from opencdarr.cr import MVP  # noqa: E402
from opencdarr.crr import PastCPA  # noqa: E402
from opencdarr.fleet import Agent, build_env  # noqa: E402
from opencdarr.ips import Particle, evolve_shard, ips_once, resample_level  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.rng import children, root_seed_sequence  # noqa: E402
from opencdarr.scenario import create_conflict  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

IMG = Path.home() / "Projects/opencdarr.github.io/docs/assets/img"
BLUE, ORANGE, GREY, RED = "#1f77b4", "#ff7f0e", "0.55", "#d62728"

# The shell ladder and the run, matching examples/handbook/rare_event_ips_illustrated.ipynb.
LEVELS = [150, 135, 122, 112, 104, 97, 90, 82, 74, 68, 63, 59, 56, 54, 52, 51, 50]
N_PARTICLES = 400
SEED = 0
CEILING = 900.0  # display clip for particles that never came close


def make_start() -> Particle:
    """The one starting particle: a fixed 90 deg crossing with GNSS noise, plus its rules."""
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889,
                        pos_ci95=3.0, vel_ci95=1.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0,
                           tlos=70.0, rpz=50.0, side=1)
    agents = [Agent(own, M600), Agent(intr, M600)]
    env = build_env(agents, rpz=50.0, t_lookahead=60.0, dt=0.5,
                    detector=StateBased(), resolver=MVP(margin=1.05),
                    recovery=PastCPA(bouncing_guard=True), navigation=GnssNavigation(),
                    done_timeout=10.0)
    return Particle(env=env, state=env.initial_state(agents))


START = make_start()


def build_initial(seq: np.random.SeedSequence) -> Particle:
    """One IPS particle. Geometry is fixed here, so the seed feeds the forward noise, not this."""
    return START


def traced_run() -> tuple[list[np.ndarray], list[float]]:
    """``ips_once``, with each particle's achieved running-minimum recorded per shell."""
    init_seq, evolve_seq = children(root_seed_sequence(SEED), 0, 2)
    particles = [build_initial(s) for s in children(init_seq, 0, N_PARTICLES)]
    level_seqs = children(evolve_seq, 0, len(LEVELS))

    achieved: list[np.ndarray] = []
    survival: list[float] = []
    for k, target in enumerate(LEVELS):
        sub = children(level_seqs[k], 0, N_PARTICLES + 1)
        evolved = evolve_shard(particles, target, sub[:N_PARTICLES])
        achieved.append(np.array([p.state.min_sep for p in evolved]))
        fraction, particles, _ = resample_level(
            evolved, target, N_PARTICLES, sub[N_PARTICLES])
        survival.append(fraction)
        if not particles:
            raise RuntimeError(f"the ladder collapsed at shell {k} ({target} m)")
    return achieved, survival


def figure(out: Path, achieved: list[np.ndarray], survival: list[float]) -> None:
    shells = np.array(LEVELS, dtype=float)
    rng = np.random.default_rng(SEED)
    fig, (a_pop, a_prod) = plt.subplots(1, 2, figsize=(9.6, 4.4))

    for k, (vals, d) in enumerate(zip(achieved, shells, strict=True)):
        x = k + 1 + rng.uniform(-0.28, 0.28, vals.size)
        y = np.minimum(vals, CEILING)
        hit = vals <= d
        a_pop.plot(x[hit], y[hit], ".", color=BLUE, ms=2.4, alpha=0.5, zorder=2)
        a_pop.plot(x[~hit], y[~hit], ".", color=RED, ms=2.4, alpha=0.5, zorder=2)
    a_pop.step(np.arange(1, len(shells) + 1), shells, where="mid",
               color="k", lw=1.4, zorder=3)
    a_pop.set_yscale("log")
    a_pop.set_ylim(45, CEILING * 1.15)
    a_pop.set_xlim(0.3, len(shells) + 0.7)
    a_pop.set_xticks(range(1, len(shells) + 1, 2))
    a_pop.set_yticks([50, 60, 80, 100, 150, 250, 500, 900])
    a_pop.set_yticklabels(["50", "60", "80", "100", "150", "250", "500", ">900"])
    a_pop.set_xlabel("shell index")
    a_pop.set_ylabel("running-minimum separation [m]")
    a_pop.set_title("The population, shell by shell", fontsize=10)
    a_pop.set_box_aspect(1)

    cumulative = np.cumprod(survival)
    k = np.arange(1, len(survival) + 1)
    a_prod.plot(k, survival, color=ORANGE, lw=1.2, ls="--", marker=".", ms=4, zorder=2,
                label="per shell, $S_k/N$")
    a_prod.plot(k, cumulative, color=BLUE, lw=1.6, marker="o", ms=3.5, zorder=3,
                label=r"running product, $\prod_{j\leq k} S_j/N$")
    a_prod.axhline(1.0, color=GREY, lw=0.6, zorder=1)
    a_prod.set_yscale("log")
    a_prod.set_xlim(0.3, len(shells) + 1.4)
    a_prod.set_xticks(range(1, len(shells) + 1, 2))
    a_prod.set_ylim(cumulative[-1] / 6, 3.0)
    a_prod.set_xlabel("shells crossed")
    a_prod.set_ylabel("survival fraction, and their running product")
    a_prod.set_title("Ordinary fractions, rare product", fontsize=10)
    exponent = int(np.floor(np.log10(cumulative[-1])))
    mantissa = cumulative[-1] / 10.0**exponent
    a_prod.text(0.66, 0.12, rf"$\hat P = {mantissa:.1f} \times 10^{{{exponent}}}$",
                transform=a_prod.transAxes, color=BLUE, fontsize=9, ha="left")
    a_prod.legend(frameon=False, fontsize=8, loc="lower left")
    a_prod.set_box_aspect(1)

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("wrote", out)


def main() -> None:
    achieved, survival = traced_run()
    reference = ips_once(build_initial, LEVELS, N_PARTICLES, root_seed_sequence(SEED))
    assert np.allclose(survival, reference.survival), "the traced loop drifted from ips_once"
    print(f"P(LoS) = {float(np.prod(survival)):.2e}  (ips_once: {reference.prob:.2e})")
    IMG.mkdir(parents=True, exist_ok=True)
    figure(IMG / "rare-event-ips-ladder.png", achieved, survival)


if __name__ == "__main__":
    main()
