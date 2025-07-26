fn main() {
    // Tell cargo to re-run this build script if linker.ld changes
    println!("cargo:rerun-if-changed=linker.ld");
}
