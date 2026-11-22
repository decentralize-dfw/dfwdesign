#!/usr/bin/env bash
# Tüm üretim çıktılarını yeniden üretir (Rev C).
set -e
cd "$(dirname "$0")"
python3 00_decode.py      # GLB -> seat.stl
python3 sec2.py           # orijinal dilim konturları -> sections.pkl
python3 vol.py            # dilimlerden işaretli uzaklık hacmi -> vol.npz
python3 build.py          # üst gövde 18° yatırılıp uzatılarak yeniden dilimleme, oyma, çubuklar, omurga
python3 check.py          # somun çakışması / et kalınlığı / omurga çakışması
python3 mesh3d.py         # chair.glb
python3 mesh_parts.py     # parça parça GLB + meta (patlatma animasyonu için)
python3 fab.py            # DXF, yerleşim, BOM.csv, summary.json -> out/
python3 ergo.py           # ergonomi.png
