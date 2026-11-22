#!/usr/bin/env bash
# Tüm üretim çıktılarını yeniden üretir (Rev B).
set -e
cd "$(dirname "$0")"
python3 00_decode.py      # GLB -> seat.stl
python3 sec2.py           # orijinal dilim konturları -> sections.pkl
python3 build.py          # kaburgalar (orijinal kontur + kova oyma + kıvrımlı ekler), çubuklar, omurga -> ribs_final.pkl
python3 check.py          # somun çakışması / et kalınlığı / omurga çakışması kontrolü
python3 mesh3d.py         # chair.glb
EXPLODE=0.15 python3 mesh3d.py
python3 fab.py            # DXF, yerleşim, BOM.csv, summary.json -> out/
python3 ergo.py           # ergonomi.png
