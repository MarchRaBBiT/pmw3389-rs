// build.rs
use std::{env, fs, io, path::PathBuf};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(pmw3389_has_srom)");
    // firmware ディレクトリ内の SROM をターゲットディレクトリへコピー
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let srom_src = PathBuf::from("firmware/pmw3389_srom.bin");
    let srom_dst = out.join("pmw3389_srom.bin");

    match fs::copy(&srom_src, &srom_dst) {
        Ok(_) => {
            println!("cargo:rustc-cfg=pmw3389_has_srom");
        }
        Err(err) if err.kind() == io::ErrorKind::NotFound => {
            println!("cargo:warning=firmware/pmw3389_srom.bin not found; skipping copy");
        }
        Err(err) => panic!("Failed to copy pmw3389_srom.bin: {err}"),
    }

    println!("cargo:rerun-if-changed=firmware/pmw3389_srom.bin");
}
