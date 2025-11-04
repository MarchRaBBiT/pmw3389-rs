use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let srom_src = PathBuf::from("firmware/pmw3389_srom.bin");
    let srom_dst = out.join("pmw3389_srom.bin");

    match fs::copy(&srom_src, &srom_dst) {
        Ok(_) => {}
        Err(err) => {
            if srom_src.exists() {
                panic!("Failed to copy pmw3389_srom.bin: {err}");
            } else {
                fs::write(&srom_dst, &[])
                    .expect("Failed to create placeholder pmw3389_srom.bin");
                println!(
                    "cargo:warning=pmw3389_srom.bin not found; firmware upload will be skipped"
                );
            }
        }
    }

    println!("cargo:rerun-if-changed=firmware/pmw3389_srom.bin");
}
