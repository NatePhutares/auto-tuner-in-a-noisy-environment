#!/bin/sh
# run_test.sh — build the C auto-tuner for the PC and compare it with autotuner.py.
#   sh run_test.sh ../../python_prototype/wav_files/introduce.wav            (nearest mode)
#   sh run_test.sh ../../python_prototype/wav_files/introduce.wav C4         (fixed mode, target C4)
# Needs gcc (Windows: MSYS2 / WSL) and python with numpy + scipy.
set -e
WAV=$1; TARGET=$2
gcc -O2 -std=c11 -Wall -Wextra -Wdouble-promotion -I../App/Inc ../App/Src/at_*.c ../App/Src/autotuner.c at_pc_main.c -lm -o at_pc
if [ -n "$TARGET" ]; then
  python make_golden.py "$WAV" gold --target "$TARGET"; ./at_pc gold/input_q15.raw out_c.raw hops_c.csv fixed "$TARGET"
else
  python make_golden.py "$WAV" gold; ./at_pc gold/input_q15.raw out_c.raw hops_c.csv
fi
python compare.py gold out_c.raw hops_c.csv
python -c "import numpy as np, scipy.io.wavfile as w; w.write('out_c.wav', 16000, np.fromfile('out_c.raw', '<i2'))"
echo "C output -> out_c.wav"
