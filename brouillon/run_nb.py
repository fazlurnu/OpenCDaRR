"""Execute prob_ftr_velocity_covariance.ipynb in place, logging progress per cell.

Run from this directory so the notebook's on-disk cache (``.opencdarr_cache``) is found:

    setsid nohup python -u run_nb.py > nb_run.log 2>&1 < /dev/null &

Every estimate inside the notebook is cached, so an interrupted run resumes where it stopped.
"""

from __future__ import annotations

import time

import nbformat
from nbclient import NotebookClient

NB = "prob_ftr_velocity_covariance.ipynb"


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    client = NotebookClient(
        nb, timeout=None, kernel_name="python3", resources={"metadata": {"path": "."}}
    )
    t0 = time.time()
    n_code = sum(1 for c in nb.cells if c.cell_type == "code")
    done = 0

    # Persist after every cell, so a kill leaves the outputs computed so far on disk.
    original = client.execute_cell

    def traced(cell, index, *args, **kwargs):
        nonlocal done
        result = original(cell, index, *args, **kwargs)
        if cell.cell_type == "code":
            done += 1
            print(f"[{time.time() - t0:7.0f}s] cell {done}/{n_code} done", flush=True)
            nbformat.write(nb, NB)
        return result

    client.execute_cell = traced
    try:
        with client.setup_kernel():
            client.execute(cleanup_kc=False)
        print(f"OK: executed in {time.time() - t0:.0f} s", flush=True)
    except Exception as exc:  # noqa: BLE001 - the log is the report
        print(f"FAILED after {time.time() - t0:.0f} s: {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        nbformat.write(nb, NB)


if __name__ == "__main__":
    main()
