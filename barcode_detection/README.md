## How to Git

## Getting Started

```
git clone https://github.com/rafzarf/ena-me-project.git
git cd ena-me-project
git checkout [working branch]
```

Untuk working branch sendiri, silahkan pilih sesuai fitur yang ingin dikerjakan
Kita akan menggunakan git flow, jadi ada 2 branch penting yaitu: `trunk` dan `development`
`trunk` adalah branch utama atau production release, sedangkan `development` adalah branch branch pengembangan atau "next release"
Git flow sendiri mempunyai beberapa branch prefix, yaitu `feature`, `bugfix`, `release`, `hotfix`, dan `support`.

## Alur kerja

Sebelum mulai bekerja

```
git fetch -> untuk memeriksa apakah ada update terkini
git pull origin [working branch] -> untuk mengambil updatean terbaru dari repo sesuai working branch
[optional] jika ingin mengambil seluruh updatean, gunakan git pull saja
```

---

Setelah selesai bekerja

```
git add . -> untuk memasukan semua perubahan ke stagging
git commit -m "[pesan commit]" -> masuk ke commit, dan memberi nama perubahannya
```

Aturan pesan commit:

- Jelas, sesuai dengan perubahan yang dilakukan. Contoh: `menambahkan button login di landing page` jangan hanya `menambahkan button` atau `mengubah button.kt` saja. Aturannya *perubahan + fitur + spesifik*
- Ringkas, tidak terbelit-belit dan langsung to the point, sehingga orang yang melihat pesannya bisa langsung paham
- (Opsional) Deskripsi, tapi ini sulit dilakukan dan ada trik tertentu jika menggunakan Bash, strukturnya `git commit -m "(judul commit)" -m "(deskripsi commit)"`

Ketika sudah yakin dengan perubahan yang dilakukan

```
git push origin [working branch] -> mengirim perubahan ke repo sesuai working branch
```

---

Beres, jika ingin bekerja lagi, ulangi langkah-langkahnya dari awal.
