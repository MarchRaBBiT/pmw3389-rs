# pmw3389-rs

**PixArt PMW3389 光学センサー**向けの [embedded-hal 1.0](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) 対応 Rust ドライバです。ゲーミングマウスやトラックボールで広く使われているセンサーを安全かつ扱いやすい API で操作できます。

このクレートはセンサーの初期化、CPI（counts per inch）の設定、モーションデータの取得、電源モードの制御といった機能を提供します。

---

## ✨ 特徴

- `embedded-hal` 1.0 に対応した SPI ドライバ
- センサー初期化（SROM アップロードを含む）
- モーションデータの取得 (`dx`, `dy`, `squal`)
- CPI の設定（50〜16,000、50 刻み）
- 省電力モード
- 基本的なエラー処理（`Spi`, `Timeout`, `InvalidParam`, `InvalidResponse`）

---

## ⚠️ SROM ファームウェアの準備

> **重要:** このドライバを動作させるには `pmw3389_srom.bin` ファームウェアが必要です。

ライセンス上の理由から、このリポジトリにはファームウェアは含まれていません。公式の入手元から取得し、ビルド時に利用できるように配置してください。

*   **入手先:** PixArt、正規代理店、またはハードウェアベンダーから取得してください。
*   **利用方法:** 本リポジトリのサンプルでは `firmware/pmw3389_srom.bin` に配置することを想定していますが、ドライバの `init` 関数は単に `&[u8]` のスライスを受け取ります。

---

## 📦 インストール

`Cargo.toml` に以下を追加します。

```toml
[dependencies]
pmw3389 = { git = "https://github.com/MarchRaBBiT/pmw3389-rs" }
```

---

## 🚀 使い方

### 初期化

```rust
use pmw3389::Pmw3389;

// ビルド時に SROM ファームウェアを読み込む
const PMW3389_SROM: &[u8] = include_bytes!("../firmware/pmw3389_srom.bin");

// 先に SPI と Delay 実装を用意する...
let mut sensor = Pmw3389::new(spi_dev, delay);

// ファームウェア付きでセンサーを初期化
sensor.init(PMW3389_SROM)?;

// プロダクト ID を取得（0x42 のはず）
let pid = sensor.product_id()?;
```

### モーションデータの取得

```rust
let motion = sensor.read_motion()?;
defmt::info!("dx={}, dy={}, squal={}", motion.delta_x, motion.delta_y, motion.squal);
```

### CPI の設定

```rust
sensor.set_cpi(1600)?;
```

### 省電力モード

```rust
sensor.power_down()?;
```

---

## 🧪 サンプル

実行可能なコードは [`examples/`](examples) ディレクトリにあります。

* [`rp2040-zero.rs`](examples/rp2040-zero.rs): 基本的な初期化とモーション読み取り
* [`cpi.rs`](examples/cpi.rs): CPI 設定のデモ
* [`power_down.rs`](examples/power_down.rs): 省電力モードのデモ

RP2040 上で実行する場合（probe-rs 利用）:

```bash
cargo run --release --example rp2040-zero
```

補足: 既定の runner（`Cargo.toml` に設定）は `probe-rs` を用いた直接書き込みです。UF2 形式を生成したい場合は `elf2uf2-rs` をインストールし、`Cargo.toml` の `runner = "elf2uf2-rs"` 行のコメントを外してから同じコマンドを実行してください。`target/thumbv6m-none-eabi/release/examples/rp2040-zero.uf2` のようなファイルが生成され、ドラッグ＆ドロップで書き込めます。

---

## 📜 ライセンス

このプロジェクトは [MIT License](LICENSE) の下で配布されています。

---

## 🙏 謝辞

* PMW3389 センサーを提供する [PixArt Imaging](https://www.pixart.com/)
* [embedded-hal](https://github.com/rust-embedded/embedded-hal) プロジェクト
* このドライバにインスピレーションを与えてくれたコミュニティプロジェクト

