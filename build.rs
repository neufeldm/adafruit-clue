use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::cfg;

fn main() {
    let s140_v6 = cfg!(feature="nrf-s140-v6");
    let s140_v7 = cfg!(feature="nrf-s140-v7");
    std::assert!((s140_v6 && !s140_v7) || (!s140_v6 && s140_v7),
                "need to have exactly one of nrf-s140-v6 or nrf-s140-v7 feature specified");
    // Put the linker script somewhere the linker can find it
    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut out_file = File::create(out_dir.join("memory.x"))
        .unwrap();
    if cfg!(feature="nrf-s140-v6") {
        out_file.write_all(include_bytes!("memory-s140_v6.x"))
        .unwrap();
    } else if cfg!(feature="nrf-s140-v7") {
        out_file.write_all(include_bytes!("memory-s140_v7.x"))
        .unwrap();
    }
    println!("cargo:rustc-link-search={}", out_dir.display());
    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    //println!("cargo:rerun-if-changed=memory.x");
}
