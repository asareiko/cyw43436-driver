## Run on real hardware
```bash
rustup target add armv7a-none-eabi
```

## Build code into ELF:
```bash
cargo build --release
```

## Proper build configuration
```bash
cargo +nightly build --release -Z build-std=core,compiler_builtins -Z build-std-features=compiler-builtins-mem
```

## Simulate it
Need :
```bash
rustup target add aarch64-unknown-none
```