# Checklist Troubleshooting EtherCAT, Servo, dan PCB

> Gunakan secara berurutan. Jangan menjalankan motor atau mengubah konfigurasi produksi tanpa persetujuan dan prosedur keselamatan.

## 0. Keselamatan dan baseline
- [ ] Catat tanggal, commit/version program, ESI XML, firmware servo, topologi, dan nomor slave.
- [ ] Pastikan emergency stop, batas mekanis, dan area gerak aman.
- [ ] Lepaskan beban mekanis bila memungkinkan; gunakan daya/arus terbatas saat diagnosis awal.
- [ ] Simpan log sebelum melakukan reset atau perubahan.
- [ ] Bedakan hasil terukur dari dugaan; jangan menganggap WKC normal berarti motion aman.

## 1. Pemeriksaan fisik dan jaringan
- [ ] Periksa urutan IN/OUT EtherCAT, konektor, kabel, shield, dan terminasi.
- [ ] Ganti kabel dengan kabel yang telah terbukti baik; cek koneksi intermittent saat sistem diam.
- [ ] Catat link LED, speed/duplex, CRC/drop/error counter pada NIC/switch yang relevan.
- [ ] Pastikan tidak ada switch Ethernet biasa di jalur EtherCAT kecuali perangkat memang mendukung EtherCAT.
- [ ] Verifikasi catu daya slave, 0 V/common, PE/chassis, dan pemisahan kabel motor dari kabel komunikasi.

## 2. Discovery dan state EtherCAT
- [ ] Jalankan discovery tanpa mengaktifkan motion; catat jumlah slave, vendor ID, product code, revision, dan state.
- [ ] Catat AL Status Code dan pesan error setiap slave.
- [ ] Pastikan urutan slave dan ESI XML cocok dengan perangkat aktual.
- [ ] Transisikan INIT → PRE-OP → SAFE-OP → OP satu tahap pada satu waktu.
- [ ] Jika Safe-Op → Op gagal, periksa PDO size, Sync Manager, FMMU, watchdog, dan error AL sebelum retry.

## 3. WKC dan process data
- [ ] Hitung expected WKC dari konfigurasi aktual; jangan memakai angka lama tanpa verifikasi.
- [ ] Log return value `ec_receive_processdata()` dan bandingkan dengan expected WKC setiap siklus.
- [ ] Saat WKC turun, catat timestamp, slave state, cycle time, dan apakah motor/power/noise berubah.
- [ ] Panggil/rekam `ec_readstate()` setelah kegagalan; gunakan `ec_printerrors()` hanya bila tersedia pada versi/build SOEM yang dipakai (API/headers berbeda antar versi), dan simpan output error yang benar-benar diperoleh.
- [ ] Bedakan state AL EtherCAT (INIT/PRE-OP/SAFE-OP/OP dan AL Status Code) dari state machine CiA 402 servo (Statusword/Controlword); OP EtherCAT tidak berarti drive sudah enabled.
- [ ] Bedakan timeout komunikasi, WKC mismatch, dan data PDO stale.
- [ ] Verifikasi ukuran dan offset input/output PDO terhadap ESI dan hasil mapping SDO.

## 4. CiA 402 / servo
- [ ] Baca Statusword 0x6041 dan Error Code 0x603F.
- [ ] Pastikan mode operasi 0x6060 dan mode display 0x6061 sesuai (CSP/CSV/PP).
- [ ] Setelah penyebab fault, STO/enable, dan area aman diverifikasi, ikuti urutan controlword CiA 402 yang ditentukan manual drive; jangan menerapkan reset fault atau enable operation secara otomatis/unconditionally.
- [ ] Verifikasi bit statusword pada setiap transisi; jangan hanya menunggu delay tetap.
- [ ] Perlakukan Fault Reset sebagai tindakan eksplisit yang memerlukan operator/prosedur keselamatan; jangan mengulanginya otomatis saat komunikasi atau status belum valid.
- [ ] Periksa limit, STO/enable input, following error, overcurrent, overvoltage, dan encoder feedback.
- [ ] Pastikan target position/velocity, scaling, sign, dan unit tidak overflow.
- [ ] Untuk diagnosis awal, gunakan target kecil dan kecepatan rendah setelah safety check.

## 5. Distributed Clocks dan timing
- [ ] Ukur cycle time aktual dan jitter; jangan hanya mengandalkan konfigurasi nominal.
- [ ] Pastikan `ec_configdc()`/DC setup dipanggil pada urutan yang benar bila DC digunakan.
- [ ] Verifikasi Sync0/Sync1 period, shift time, dan reference clock.
- [ ] Periksa CPU load, scheduling, page faults, dan prioritas thread master.
- [ ] Jika master/drive mendukungnya, lakukan uji pembanding DC nonaktif dan aktif hanya pada mode konfigurasi yang didokumentasikan; uji DC nonaktif harus tanpa motion/enable operation dan dengan output aman.
- [ ] Jangan menonaktifkan DC, watchdog, STO, limit, atau proteksi untuk memaksa OP; catat konfigurasi asli dan prosedur pemulihan.
- [ ] Catat korelasi antara jitter, WKC drop, dan transisi state servo.

## 6. Linux/master runtime
- [ ] Catat interface jaringan yang benar, driver, link status, dan konfigurasi realtime.
- [ ] Pastikan hanya satu master mengakses interface EtherCAT.
- [ ] Periksa permission, IRQ affinity, CPU frequency governor, dan proses yang mengganggu cycle thread.
- [ ] Simpan log stdout/stderr dan konfigurasi yang dipakai; jangan mengompilasi atau menjalankan hardware tanpa otorisasi eksplisit.

## 7. PCB, kabel, dan EMC
- [ ] Verifikasi differential pair Ethernet/EtherCAT 100 Ω terkontrol sesuai stackup, bukan sekadar mengukur panjang trace.
- [ ] Pastikan pair memiliki reference plane kontinu dan tidak melintasi split/void.
- [ ] Cek return path, via transition, stub, konektor, magnetics, dan terminasi sesuai PHY/vendor reference design.
- [ ] Tempatkan decoupling dekat pin supply; cek loop arus regulator dan ground return.
- [ ] Pisahkan jalur motor/switching berarus tinggi dari PHY dan oscillator; cek shielding/chassis bonding.
- [ ] Verifikasi grounding, shield termination, chassis/PE bonding, dan isolasi galvanik sesuai persyaratan keselamatan, desain magnetics, dan reference design/vendor—jangan membuat koneksi ground tambahan secara trial-and-error.
- [ ] Periksa thermal vias, temperatur regulator/PHY/driver, dan derating komponen.
- [ ] Jika error hanya muncul saat motor aktif, lakukan uji A/B dengan shielding, kabel, grounding, dan beban yang aman, hanya setelah konfigurasi grounding disetujui.

## 8. Bukti minimum sebelum meminta bantuan
- [ ] Topologi dan foto/diagram kabel.
- [ ] Vendor/product/revision setiap slave.
- [ ] ESI XML dan hasil `ec_config_init()`/discovery.
- [ ] Expected WKC vs actual WKC beserta timestamp.
- [ ] AL status/code, CiA 402 Statusword/Error Code.
- [ ] PDO mapping lengkap dan mode operasi.
- [ ] Cycle time, jitter, DC settings, CPU/NIC details.
- [ ] Kondisi saat error: motor off/on, velocity, temperatur, panjang kabel, dan power supply.
- [ ] Commit program dan perubahan terakhir.

## Tabel diagnosis cepat

| Gejala | Bukti pertama | Isolasi aman |
|---|---|---|
| WKC lebih kecil dari expected | log WKC, state, AL code | cek kabel/slave lalu ulang discovery tanpa motion |
| Safe-Op tidak ke Op | PDO/SM/FMMU, watchdog, AL code | cocokkan ESI/PDO dan baca status slave |
| Servo Fault | 0x6041, 0x603F, STO/enable | reset fault hanya setelah penyebab dan area aman diverifikasi |
| Gerak tidak sesuai | 0x6060/0x6061, scaling, target | target kecil, tanpa beban, verifikasi satuan/sign |
| Error hanya saat motor aktif | timestamp vs motion, CRC/drop, supply noise | uji kabel/shielding/grounding dan pisahkan sumber noise |
| Jitter tinggi | cycle histogram, CPU/IRQ load | kurangi beban sistem, cek scheduling dan DC timing |

## Aturan eskalasi
- [ ] Hentikan pengujian jika ada gerak tak terkendali, fault berulang, panas abnormal, bau, atau suara mekanis tidak normal.
- [ ] Jangan menonaktifkan watchdog, STO, limit, atau proteksi hanya untuk membuat status OP tercapai.
- [ ] Dokumentasikan satu perubahan setiap eksperimen agar sebab-akibat dapat ditelusuri.
