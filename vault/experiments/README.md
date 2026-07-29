# experiments

One provenance card per run — `config + seed + code-hash -> result` — so any figure is
traceable back to the exact inputs that produced it (design-philosophy #4). Cards are the
audit trail, not the data (outputs are gitignored).

**No cards are committed, so this directory is empty in a fresh clone.** `run_one_experiment`
writes them here at run time (`card_dir` defaults to `vault/experiments`), so it fills up locally
and stays empty in git. That is the design: a card is regenerable from the config and seed it
records, so committing it would store a product rather than a source.

The runs behind the published observations are traced a different way — the observation note names
the script and the parameters itself. For the two big sweeps the raw logs *are* committed:
`scripts/cns_sweep_20260728_085447/` and `scripts/ips_rerun_20260729_072950/`, each with a
`summary.tsv`.
