//! Hardware Abstraction Layer for BCM2710 (Raspberry Pi Zero 2 W)
//!
//! This module provides low-level hardware access for the BCM2710 SoC,
//! including GPIO, SDIO/EMMC2 controller, and other peripherals.

pub mod gpio;
pub mod mmio;
pub mod sdio;
pub mod timer;

/// Base address for BCM2710 peripherals
pub const PERIPHERAL_BASE: u32 = 0x3F000000;

/// GPIO base address
pub const GPIO_BASE: u32 = PERIPHERAL_BASE + 0x200000;

/// EMMC2 (SDIO) base address
pub const EMMC2_BASE: u32 = PERIPHERAL_BASE + 0x340000;

/// System Timer base address
pub const TIMER_BASE: u32 = PERIPHERAL_BASE + 0x003000;
