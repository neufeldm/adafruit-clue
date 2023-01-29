use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put the linker script somewhere the linker can find it
    // XXX detect the s140 version and copy the appropriate memory.x file
    // XXX read version from a file instead of environment?
    let mut s140_version = env::var_os("S140_VERSION").unwrap().into_string().unwrap();
    s140_version.truncate(1);
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut out_file = File::create(out.join("memory.x")).unwrap();
    match s140_version.parse::<u8>().unwrap() {
        6 => out_file.write_all(include_bytes!("memory-s140_v6.x")).unwrap(),
        7 => out_file.write_all(include_bytes!("memory-s140_v7.x")).unwrap(),
        _ => println!("bite me!"),
    }
    println!("cargo:rustc-link-search={}", out.display());

    // Only re-run the build script when memory.x is changed,
    // instead of when any part of the source code changes.
    // XXX fixme - should probably rerun every time, or if we pull version from file
    println!("cargo:rerun-if-changed=memory.x");
}
