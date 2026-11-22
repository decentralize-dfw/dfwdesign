# Armageddon Metal Koltuk (armageddonseat-v17 → üretim revizyonu)

Metaverse için tasarlanan `armageddonseat-v17.glb` dilimli figür koltuğun, lazerle kesilip
birbirine geçerek monte edilen, cıvata detaylı metal versiyonu. Ayrıntılı rapor: `rapor.html`
(etkileşimli 3D model, ergonomi, statik, BOM, montaj sırası).

## Rev D (üç yeni yaklaşım, Rev C korunuyor)
Kural: sadece lazer kesim plakalar + M16 cıvata (pul, somun). Mesafe borusu ya da başka ek parça yok; aralık lazer kesim tarak/yarım-geçmelerden gelir. Oranlar orijinalle 1:1 (üst gövde 13° geriye yaslanmış).
- **A · Halka** — orijinal ön dilimler; orta omurga laması, omuz tarakları, ayak ve el plakaları. 32 kaburga + 7 tarak, 7 × M16. `varyantlar/A`
- **B · Hibrit** — gövde/bacaklar ön dilimli; oturak öne-arkaya uzanan 18 yan plakadan (rahat, sürekli kontur). 43 plaka, 9 × M16. `varyantlar/B`
- **C · Izgara** — iki yönlü geçme ızgara; yan plakalar durur, ön plakalar yukarıdan iner. 33 plaka, 2 × M16. `varyantlar/C`
- `rapor.html` + `models/`: dört varyant + orijinal; plastik sandalye ve oturan insanla kıyas; animasyonlu patlatma.

## Rev C
- Üst gövde kalçadan 18° geriye yatırıldı ve %25 uzatıldı; figür dilimlerinden kurulan 3B hacim yeniden 25 mm'de dilimlendi. Yükseklik orijinal gibi ~1,29 m, göğüs oturanın sırtına paralel, baş baş desteği konumunda.
- `rapor.html`: döndürülebilir model + animasyonlu patlatılmış görünüm (kesikli çizgiler: çubuk eksenleri ve kaburga→omurga yuvaları).

## Rev B
- Oturak kısaltıldı: tüm dilimler orijinal yerinde (Rev A'daki 75 mm öne kaydırma ve ara dilimler kaldırıldı).
- Kaburgalar orijinal dilim konturlarının 5 mm ofsetle kalınlaştırılmış hali; halkalar, kas çizgileri, mercek boşluklar korunuyor.
- Ergonomik oyma kova biçiminde; sırt temas kenarları kıvrımlı kaburga bantları. Tüm ekler kıvrımlı, birleşimler R10.

## Özet
| | |
|---|---|
| Malzeme | Al 6082-T651, 8 mm levha (fiber lazer / su jeti) |
| Dilim adımı | 25 mm (orijinal ile aynı), 8 mm kaburga + 17 mm paslanmaz mesafe borusu |
| Bağlantı | Oturak altında yarım-geçme yuvalı omurga laması (K01) + M12/M10/M8 A2 gergi çubukları, kep somunlar |
| Ölçü | 763 × 1288 × 1038 mm (G×Y×D) |
| Oturma | ön 437 / arka 413 mm yükseklik, 385 mm derinlik, oturak–sırt ≈104°, baş desteği ~1,15–1,25 m |
| Ağırlık | ≈57 kg (47 kg levha + 10 kg bağlantı) |
| Kesim | 74 parça, 3 levha 3000×1500 |

## Klasörler
- `model/` montajlı GLB, parça parça GLB + `chair_parts.json` (patlatma animasyonu)
- `dxf/` parça başına DXF (katmanlar: CUT, HOLES, ETCH) ve gösterge yerleşim dosyaları
- `BOM.csv`, `summary.json` malzeme listesi ve hesap özetleri
- `img/` render, ergonomi kesiti, yerleşim
- `src/` parametrik üretici; `src/run.sh` tüm çıktıları yeniden üretir

## İmalat notları
- Çubuk delikleri d + 0,8 mm; omurga yuvaları 8,6 mm (levha + 0,6), 1,5×45° giriş pahı.
  Levhayı kesimden önce ölçün; toz boya > 80 µm ise yuvayı 0,2 mm büyütün.
- Oturak ve sırt temas kenarları R2–R3 yuvarlatılmalı.
- Tork: M12 30 Nm, M10 20 Nm, M8 12 Nm, M6 5 Nm (pul + Loctite 243).
- Statik değerler ön boyutlandırmadır; seri üretimden önce prototipte EN 1728 / EN 1022 testleri önerilir.
