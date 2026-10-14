"""make_golden.py — run autotuner.py on a WAV and save reference vectors for the C port.

    python make_golden.py ../../python_prototype/wav_files/introduce.wav gold_intro            # nearest mode
    python make_golden.py ../../python_prototype/wav_files/introduce.wav gold_intro_C4 --target C4   # fixed mode

Writes <dir>/input_q15.raw (what the C program reads), output_q15.raw and hops.csv (Python results).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "python_prototype"))
import autotuner as at  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("wav")
ap.add_argument("outdir")
ap.add_argument("--target", help="fixed mode with this target, e.g. C4")
args = ap.parse_args()
cfg = at.Config(mode=at.MODE_FIXED, target_midi=at.parse_note(args.target)) if args.target else at.Config()
at.export_golden(args.outdir, at.load_wav(args.wav, cfg.fs), cfg)
print("golden vectors ->", args.outdir)
