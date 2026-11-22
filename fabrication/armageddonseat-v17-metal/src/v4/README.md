# Rev D üreticisi (A · Halka, B · Hibrit, C · Kabuk)

Kural: sadece lazer kesim plakalar + M16 cıvata/pul/somun. Mesafe borusu yok; aralık ve hiza lazer kesim taraklar ve yarım-geçme yuvalarla sağlanır.

Çalıştırma (`../` içindeki Rev C adımları önce çalışmış olmalı: `sections.pkl`, `../v3/vol.npz`):

    python3 mkfill.py && python3 mksolid.py
    python3 varA.py && python3 varB.py && python3 varC2.py
    for v in A B C; do python3 interf.py $v; python3 clash.py $v; done
    python3 convC.py      # Rev C'yi aynı patlatma formatına çevirir

Çıktı: `out/<A|B|C|RC>/model.glb`, `meta.json` (patlatma hareketleri, yuva/eksen çizgileri, montaj sırası), `BOM.csv`, `dxf/`.
