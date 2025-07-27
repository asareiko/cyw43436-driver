//! # CYW43436 WiFi Driver
//!
//! A bare-metal WiFi driver for the CYW43436 chip used in Raspberry Pi Zero 2 W.
//!
//! This driver provides complete WiFi functionality including:
//! - Network scanning
//! - WPA/WPA2 authentication with 4-way handshake
//! - Connection management
//! - Multiple security protocols (Open, WEP, WPA/WPA2)
//!
//! ## Features
//!
//! - `defmt`: Enable defmt logging support (enabled by default)
//! - `embassy`: Enable Embassy async runtime integration
//! - `std`: Enable std support for testing
//!
//! ## Example
//!
//! ```rust,no_run
//! use cyw43436_driver::{Cyw43436Driver, ConnectionParams, SecurityType};
//!
//! // Initialize the driver
//! let mut wifi_driver = Cyw43436Driver::new();
//! wifi_driver.init().unwrap();
//!
//! // Scan for networks
//! let scan_results = wifi_driver.scan().unwrap();
//!
//! // Connect to a WPA2 network
//! let params = ConnectionParams::new("MyNetwork", "password123", SecurityType::Wpa2).unwrap();
//! let state = wifi_driver.connect(params).unwrap();
//! ```

#![no_std]
#![deny(unsafe_code)]
#![warn(
    missing_docs,
    clippy::all,
    clippy::pedantic,
    clippy::cargo
)]

pub mod hal;

// Re-export the main driver and types
pub use cyw43436::{
    Cyw43436Driver, Cyw43436Error, Cyw43436Status,
    ScanResult, SecurityType, ConnectionState,
    WifiCredentials, ConnectionParams
};

// Re-export HAL components for advanced users
pub use hal::{
    gpio::GpioController,
    sdio::{SdioController, SdioCommand, SdioCommandFlags, SdioError, SdioResponse},
    timer::SystemTimer,
    mmio::Mmio,
};

mod cyw43436;
