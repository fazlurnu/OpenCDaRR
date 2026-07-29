# papers

**A PDF library, not a set of notes — and it is gitignored.** `vault/papers/` is excluded in
`.gitignore` because the PDFs may be confidential, so this README is the only file here that a
cloner sees. Locally it holds the source papers, grouped into `my-paper/`, `rare-event-sim/`,
`artificial-potential-field/`, and loose references (ADS-B reception probability, DORCA, wind on
drones, pilot response models).

The literature notes this file originally promised — one per paper, e.g. Blom & Bakker (interacting
particle system) or Schaefer & Jonas (ADS-B noise) — were never written as separate files. Where a
paper matters to a decision, it is cited in place: in the relevant ADR under
[`../decisions/`](../decisions/), or in the derivation that transcribes its equations under
[`../derivations/`](../derivations/).

`my-paper/writing-style.md` is the exception and does live here: it is the distilled register of
the author's own five papers, and one of the two sources behind
`~/Projects/opencdarr.github.io/STYLE.md`.
