//! Basic WiFi connection example for CYW43436 driver
//!
//! This example demonstrates how to use the CYW43436 driver library
//! in a bare-metal Raspberry Pi Zero 2 W project.

#![no_std]
#![no_main]

use cyw43436_driver::{
    Cyw43436Driver, ConnectionParams, SecurityType, ConnectionState
};

#[cfg(feature = "defmt")]
use defmt_rtt as _;

#[cfg(feature = "defmt")]
use panic_probe as _;

#[no_mangle]
pub extern "C" fn _start() -> ! {
    main()
}

fn main() -> ! {
    // Initialize the WiFi driver
    let mut wifi_driver = Cyw43436Driver::new();

    match wifi_driver.init() {
        Ok(()) => defmt::info!("WiFi driver initialized successfully"),
        Err(e) => {
            defmt::error!("Failed to initialize WiFi driver: {:?}", e);
            panic!("WiFi initialization failed");
        }
    }

    // Scan for available networks
    defmt::info!("Scanning for WiFi networks...");
    match wifi_driver.scan() {
        Ok(scan_results) => {
            defmt::info!("Found {} networks:", scan_results.len());
            for result in &scan_results {
                defmt::info!("  {}", result);
            }
        }
        Err(e) => {
            defmt::error!("WiFi scan failed: {:?}", e);
        }
    }

    // Connect to a specific network
    let ssid = "YourNetworkName";
    let password = "YourPassword";

    match ConnectionParams::new(ssid, password, SecurityType::Wpa2) {
        Ok(params) => {
            defmt::info!("Connecting to network: {}", ssid);

            match wifi_driver.connect(params) {
                Ok(ConnectionState::Connected) => {
                    defmt::info!("Successfully connected to WiFi!");

                    // Your application logic here
                    loop {
                        // Check connection status periodically
                        if let Ok(state) = wifi_driver.get_connection_state() {
                            match state {
                                ConnectionState::Connected => {
                                    defmt::debug!("Still connected");
                                    // Do networking tasks here
                                }
                                _ => {
                                    defmt::warn!("Connection lost: {:?}", state);
                                    break;
                                }
                            }
                        }

                        // Sleep for a bit
                        cortex_m::asm::delay(1_000_000); // ~1 second at 1MHz
                    }
                }
                Ok(state) => {
                    defmt::error!("Connection failed with state: {:?}", state);
                }
                Err(e) => {
                    defmt::error!("Connection error: {:?}", e);
                }
            }
        }
        Err(e) => {
            defmt::error!("Failed to create connection parameters: {:?}", e);
        }
    }

    // Disconnect gracefully
    if let Err(e) = wifi_driver.disconnect() {
        defmt::warn!("Failed to disconnect cleanly: {:?}", e);
    }

    loop {
        cortex_m::asm::wfi(); // Wait for interrupt
    }
}
