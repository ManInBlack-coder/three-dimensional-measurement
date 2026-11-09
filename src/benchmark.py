"""Accuracy on many simulated scans, with and without real-world errors.

  python benchmark.py                 # all scenarios, 5 runs each
  python benchmark.py --korda 10 --stsenaarium "servo lõtk 1°"

Run it before and after changing the analysis to see whether the change helped.
"""

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")  # one numpy thread per process; processes run in parallel

import argparse
import contextlib
import io
import tempfile
import time
from multiprocessing import Pool

import numpy as np

from processing.analysis import analyze
from ruumala import DEFAULTS
from simulation.simulator import simulate

ALL_ERRORS = dict(noise=0.02, noise_rel=0.01, backlash=1.0, jitter=0.5, scale=0.03, beam=2.0)
SCENARIOS = {
    "ideaalne": {},
    "müra 2 cm + 1 %": dict(noise=0.02, noise_rel=0.01),
    "servo lõtk 1°": dict(backlash=1.0),
    "servo värin 0.5°": dict(jitter=0.5),
    "servo skaala 3 %": dict(scale=0.03),
    "kiir 2°": dict(beam=2.0),
    "kõik koos": ALL_ERRORS,
}


def run(job):
    name, errors, seed, n_pos = job
    cfg = dict(DEFAULTS)
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out:
        truth = simulate(d, cfg, n_pos, seed, errors)
        try:
            res = analyze(d, cfg)
        except Exception as e:  # a crash is a result too
            return name, truth, None, f"{type(e).__name__}: {e}"
    return name, truth, res, "KONTROLLI" in out.getvalue()


def main():
    p = argparse.ArgumentParser(description="Täpsus simuleeritud skaneeringutel")
    p.add_argument("--korda", type=int, default=5, help="mitu juhuslikku katset stsenaariumi kohta")
    p.add_argument("--asukohti", type=int, default=3, choices=[1, 2, 3, 4])
    p.add_argument("--stsenaarium", choices=list(SCENARIOS), help="ainult see stsenaarium")
    args = p.parse_args()

    names = [args.stsenaarium] if args.stsenaarium else list(SCENARIOS)
    jobs = [(n, SCENARIOS[n], seed, args.asukohti) for n in names for seed in range(1, args.korda + 1)]
    results, t0 = [], time.time()
    with Pool() as pool:
        for r in pool.imap_unordered(run, jobs):
            results.append(r)
            print(f"  {len(results)}/{len(jobs)} valmis ({time.time() - t0:.0f} s): {r[0]}", flush=True)
    print()

    print(f"{args.korda} katset, {args.asukohti} asukohta\n")
    print(f"{'stsenaarium':<18} {'ruumala viga':>13} {'halvim':>8} {'mööbel':>8} {'tõde':>6}  hoiatus  krahh")
    for n in names:
        rows = [r for r in results if r[0] == n]
        ok = [r for r in rows if r[2] is not None]
        crashed = len(rows) - len(ok)
        if not ok:
            print(f"{n:<18} kõik {crashed} katset jooksid kokku: {rows[0][3]}")
            continue
        err = np.array([(r[2]["room"] / r[1]["room"] - 1) * 100 for r in ok])
        furn = np.mean([r[2]["objects"] for r in ok])
        warn = sum(1 for r in ok if r[3])
        print(f"{n:<18} {err.mean():+12.1f}% {err[np.argmax(np.abs(err))]:+7.1f}% "
              f"{furn:7.2f}  {ok[0][1]['objects']:5.2f}   {warn:>2}/{len(ok)}    {crashed:>2}")
    print("\nruumala viga = keskmine, halvim = suurim kõrvalekalle; mööbel ja tõde m³-tes")


if __name__ == "__main__":
    main()
