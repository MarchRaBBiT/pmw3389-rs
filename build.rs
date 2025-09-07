// build.rs
use std::{env, fs, path::PathBuf};

fn main() {
    // firmware ディレクトリ内の SROM をターゲットディレクトリへコピー
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let srom_src = PathBuf::from("firmware/pmw3389_srom.bin");
    let srom_dst = out.join("pmw3389_srom.bin");

    fs::copy(&srom_src, &srom_dst).expect("Failed to copy pmw3389_srom.bin");

    println!("cargo:rerun-if-changed=firmware/pmw3389_srom.bin");
}
