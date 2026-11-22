# Armageddon Metal Koltuk (armageddonseat-v17 → üretim revizyonu)

Metaverse için tasarlanan `armageddonseat-v17.glb` dilimli figür koltuğun, lazerle kesilip
birbirine geçerek monte edilen, cıvata detaylı metal versiyonu. Ayrıntılı rapor: `rapor.html`
(etkileşimli 3D model, ergonomi, statik, BOM, montaj sırası).

## Özet
| | |
|---|---|
| Malzeme | Al 6082-T651, 8 mm levha (fiber lazer / su jeti) |
| Dilim adımı | 25 mm (orijinal ile aynı), 8 mm kaburga + 17 mm paslanmaz mesafe borusu |
| Bağlantı | Oturak altında yarım-geçme yuvalı omurga laması (K01) + M12/M10/M8 A2 gergi çubukları, kep somunlar |
| Ölçü | 776 × 1164 × 1013 mm (G×Y×D) |
| Oturma | ön 435 / arka 408 mm yükseklik, 420 mm derinlik, 430 mm genişlik, oturak–sırt ≈105° |
| Ağırlık | ≈48 kg (38 kg levha + 10 kg bağlantı) |
| Kesim | 69 parça, 2 levha 3000×1500 |

## Klasörler
- `model/` montajlı ve patlatılmış GLB
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
