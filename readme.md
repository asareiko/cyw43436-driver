# CYW43436 WiFi Driver

A bare-metal WiFi driver for the CYW43436 chip used in Raspberry Pi Zero 2 W. This driver provides complete WiFi functionality without requiring an operating system.

## ✨ Features

- **Complete WiFi Stack**: Scanning, connection, and authentication
- **WPA/WPA2 Security**: Full 4-way handshake implementation
- **Multiple Protocols**: Open, WEP, WPA/WPA2 support
- **Bare Metal**: No OS dependencies, `no_std` compatible
- **Memory Efficient**: Heap-free implementation using `heapless`
- **Production Ready**: Comprehensive error handling and timeouts

## 🚀 Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
cyw43436-driver = "0.1.0"
```

Basic usage:

```rust
use cyw43436_driver::{Cyw43436Driver, ConnectionParams, SecurityType};

// Initialize the driver
let mut wifi_driver = Cyw43436Driver::new();
wifi_driver.init()?;

// Scan for networks
let networks = wifi_driver.scan()?;

// Connect to WPA2 network
let params = ConnectionParams::new("MyWiFi", "password", SecurityType::Wpa2)?;
wifi_driver.connect(params)?;
```

## 📋 Requirements

- **Hardware**: Raspberry Pi Zero 2 W
- **Target**: `armv7a-rpi-none-eabihf` (bare-metal ARM)
- **Rust**: 1.70+ with `no_std` support

## 🔧 Features

- `defmt` (default): Enable defmt logging
- `embassy`: Embassy async runtime integration
- `std`: Enable for host testing

## 📖 Examples

See the [`examples/`](examples/) directory for complete usage examples:

- [`basic_connection.rs`](examples/basic_connection.rs): Simple WiFi connection
- More examples coming soon...

## 🏗️ Integration in Your Project

### Option 1: As a Cargo Dependency

```toml
# In your bare-metal project's Cargo.toml
[dependencies]
cyw43436-driver = { path = "../cyw43436-driver" }
# Or from a git repository:
# cyw43436-driver = { git = "https://github.com/yourusername/cyw43436-driver" }
```

### Option 2: As a Git Submodule

```bash
# In your project root
git submodule add https://github.com/yourusername/cyw43436-driver.git drivers/cyw43436
```

Then in your `Cargo.toml`:

```toml
[dependencies]
cyw43436-driver = { path = "drivers/cyw43436" }
```

## 🎯 Target Configuration

Ensure your project uses the custom target for Raspberry Pi Zero 2 W:

```toml
# In .cargo/config.toml
[build]
target = "armv7a-rpi-none-eabihf"

[target.armv7a-rpi-none-eabihf]
runner = "qemu-system-arm -M raspi2b -kernel"
```

## 📚 API Documentation

### Core Types

- `Cyw43436Driver`: Main WiFi driver
- `ConnectionParams`: Network connection configuration  
- `ScanResult`: WiFi network information
- `SecurityType`: WiFi security protocols
- `ConnectionState`: Connection status

### Key Methods

- `init()`: Initialize the WiFi chip
- `scan()`: Scan for available networks
- `connect()`: Connect to a network
- `disconnect()`: Disconnect from network
- `get_connection_state()`: Check connection status

## 🛠️ Hardware Setup

The driver expects standard Raspberry Pi Zero 2 W pin configuration:
- SDIO pins for WiFi communication
- GPIO pins for power management
- Proper power sequencing

## 🧪 Testing

Run tests with:

```bash
# Unit tests (requires std feature)
cargo test --features std

# Build for target
cargo build --target armv7a-rpi-none-eabihf --release

# Run example
cargo run --example basic_connection --target armv7a-rpi-none-eabihf
```

## 📄 License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📞 Support

- 📖 [Documentation](https://docs.rs/cyw43436-driver)
- 🐛 [Issue Tracker](https://github.com/yourusername/cyw43436-driver/issues)
- 💬 [Discussions](https://github.com/yourusername/cyw43436-driver/discussions)
