//! GPIO controller for BCM2710
//! 
//! Provides control over GPIO pins for CYW43436 power management and SDIO interface

use super::mmio::Mmio;
use super::GPIO_BASE;
use bitflags::bitflags;

/// GPIO function select register offsets
const GPFSEL0: u32 = 0x00;
const GPFSEL1: u32 = 0x04;
const GPFSEL2: u32 = 0x08;
const GPFSEL3: u32 = 0x0C;
const GPFSEL4: u32 = 0x10;
const GPFSEL5: u32 = 0x14;

/// GPIO pin output set registers
const GPSET0: u32 = 0x1C;
const GPSET1: u32 = 0x20;

/// GPIO pin output clear registers
const GPCLR0: u32 = 0x28;
const GPCLR1: u32 = 0x2C;

/// GPIO pin level registers
const GPLEV0: u32 = 0x34;
const GPLEV1: u32 = 0x38;

/// GPIO pull-up/down control register
const GPPUD: u32 = 0x94;
const GPPUDCLK0: u32 = 0x98;
const GPPUDCLK1: u32 = 0x9C;

/// GPIO pin function modes
#[derive(Debug, Clone, Copy)]
pub enum GpioFunction {
    Input = 0b000,
    Output = 0b001,
    Alt0 = 0b100,
    Alt1 = 0b101,
    Alt2 = 0b110,
    Alt3 = 0b111,
    Alt4 = 0b011,
    Alt5 = 0b010,
}

/// GPIO pull-up/down control
#[derive(Debug, Clone, Copy)]
pub enum GpioPull {
    None = 0b00,
    PullDown = 0b01,
    PullUp = 0b10,
}

bitflags! {
    /// GPIO pin numbers for CYW43436 control
    pub struct CywGpioPins: u32 {
        /// WiFi power enable (example pin - adjust based on actual hardware)
        const WIFI_PWR_EN = 1 << 32;
        /// WiFi reset pin
        const WIFI_RESET = 1 << 33;
        /// SDIO data pins (GPIO 34-39 for SDIO on EMMC2)
        const SDIO_DAT0 = 1 << 34;
        const SDIO_DAT1 = 1 << 35;
        const SDIO_DAT2 = 1 << 36;
        const SDIO_DAT3 = 1 << 37;
        const SDIO_CLK = 1 << 38;
        const SDIO_CMD = 1 << 39;
    }
}

/// GPIO controller for BCM2710
pub struct GpioController {
    base: u32,
}

impl GpioController {
    /// Create a new GPIO controller
    pub const fn new() -> Self {
        Self { base: GPIO_BASE }
    }

    /// Set GPIO pin function
    pub fn set_function(&self, pin: u8, function: GpioFunction) {
        if pin >= 54 {
            return; // Invalid pin number
        }

        let reg_index = pin / 10;
        let bit_offset = (pin % 10) * 3;
        
        let reg_offset = match reg_index {
            0 => GPFSEL0,
            1 => GPFSEL1,
            2 => GPFSEL2,
            3 => GPFSEL3,
            4 => GPFSEL4,
            5 => GPFSEL5,
            _ => return,
        };

        let reg = unsafe { Mmio::<u32>::new(self.base + reg_offset) };
        let mask = !(0b111 << bit_offset);
        let value = (function as u32) << bit_offset;
        
        reg.modify(|current| (current & mask) | value);
    }

    /// Set GPIO pin high
    pub fn set_high(&self, pin: u8) {
        if pin >= 54 {
            return;
        }

        let reg_offset = if pin < 32 { GPSET0 } else { GPSET1 };
        let bit = pin % 32;
        
        let reg = unsafe { Mmio::<u32>::new(self.base + reg_offset) };
        reg.write(1 << bit);
    }

    /// Set GPIO pin low
    pub fn set_low(&self, pin: u8) {
        if pin >= 54 {
            return;
        }

        let reg_offset = if pin < 32 { GPCLR0 } else { GPCLR1 };
        let bit = pin % 32;
        
        let reg = unsafe { Mmio::<u32>::new(self.base + reg_offset) };
        reg.write(1 << bit);
    }

    /// Read GPIO pin level
    pub fn read_level(&self, pin: u8) -> bool {
        if pin >= 54 {
            return false;
        }

        let reg_offset = if pin < 32 { GPLEV0 } else { GPLEV1 };
        let bit = pin % 32;
        
        let reg = unsafe { Mmio::<u32>::new(self.base + reg_offset) };
        (reg.read() & (1 << bit)) != 0
    }

    /// Set GPIO pull-up/down control
    pub fn set_pull(&self, pin: u8, pull: GpioPull) {
        if pin >= 54 {
            return;
        }

        // Set pull-up/down control
        let pud_reg = unsafe { Mmio::<u32>::new(self.base + GPPUD) };
        pud_reg.write(pull as u32);

        // Wait 150 cycles (required by BCM2710)
        for _ in 0..150 {
            core::hint::spin_loop();
        }

        // Enable pull-up/down control for the pin
        let clk_reg_offset = if pin < 32 { GPPUDCLK0 } else { GPPUDCLK1 };
        let bit = pin % 32;
        let clk_reg = unsafe { Mmio::<u32>::new(self.base + clk_reg_offset) };
        clk_reg.write(1 << bit);

        // Wait 150 cycles
        for _ in 0..150 {
            core::hint::spin_loop();
        }

        // Clear the control signal
        pud_reg.write(0);
        clk_reg.write(0);
    }

    /// Configure SDIO pins for EMMC2 controller
    pub fn configure_sdio_pins(&self) {
        // Configure SDIO pins for Alt3 function (EMMC2 on BCM2710)
        // GPIO 34-39 are used for SDIO on EMMC2
        for pin in 34..=39 {
            self.set_function(pin, GpioFunction::Alt3);
            self.set_pull(pin, GpioPull::PullUp);
        }
    }

    /// Initialize CYW43436 power control pins
    pub fn init_cyw43436_power_pins(&self) {
        // Note: Actual pin numbers depend on the specific board design
        // This is a template - adjust pin numbers based on actual hardware
        
        // Configure power enable pin as output
        // self.set_function(POWER_PIN, GpioFunction::Output);
        // self.set_low(POWER_PIN); // Start with WiFi powered off
        
        // Configure reset pin as output  
        // self.set_function(RESET_PIN, GpioFunction::Output);
        // self.set_low(RESET_PIN); // Hold in reset initially
    }

    /// Power on the CYW43436 chip
    pub fn power_on_cyw43436(&self) {
        // Power sequence for CYW43436
        // 1. Enable power
        // 2. Wait for stabilization
        // 3. Release reset
        
        // Implementation depends on actual hardware connections
        defmt::info!("Powering on CYW43436...");
        
        // Example power-on sequence (adjust for actual hardware)
        // self.set_high(POWER_PIN);
        // Wait for power stabilization
        // for _ in 0..100000 { core::hint::spin_loop(); }
        // self.set_high(RESET_PIN);
    }

    /// Power off the CYW43436 chip
    pub fn power_off_cyw43436(&self) {
        defmt::info!("Powering off CYW43436...");
        
        // Example power-off sequence (adjust for actual hardware)
        // self.set_low(RESET_PIN);
        // self.set_low(POWER_PIN);
    }
}
