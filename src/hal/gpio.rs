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
        /// WiFi power enable (GPIO 2 - example, adjust based on actual hardware)
        const WIFI_PWR_EN = 1 << 2;
        /// WiFi reset pin (GPIO 3)
        const WIFI_RESET = 1 << 3;
        /// SDIO data pins (GPIO 34-39 for SDIO on EMMC2)
        const SDIO_DAT0 = 1 << 22;  // GPIO 22
        const SDIO_DAT1 = 1 << 23;  // GPIO 23
        const SDIO_DAT2 = 1 << 24;  // GPIO 24
        const SDIO_DAT3 = 1 << 25;  // GPIO 25
        const SDIO_CLK = 1 << 26;   // GPIO 26
        const SDIO_CMD = 1 << 27;   // GPIO 27
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

        // BCM2710 pull-up/down control sequence
        let pud_reg = unsafe { Mmio::<u32>::new(self.base + GPPUD) };
        let clk_reg = unsafe { 
            if pin < 32 { 
                Mmio::<u32>::new(self.base + GPPUDCLK0) 
            } else { 
                Mmio::<u32>::new(self.base + GPPUDCLK1) 
            }
        };

        // Step 1: Write to GPPUD to set the required control signal
        pud_reg.write(pull as u32);
        
        // Step 2: Wait 150 cycles (this provides the required setup time)
        for _ in 0..150 {
            unsafe { core::arch::asm!("nop") };
        }
        
        // Step 3: Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads
        let bit = pin % 32;
        clk_reg.write(1 << bit);
        
        // Step 4: Wait 150 cycles (this provides the required hold time)
        for _ in 0..150 {
            unsafe { core::arch::asm!("nop") };
        }
        
        // Step 5: Write to GPPUD to remove the control signal
        pud_reg.write(0);
        
        // Step 6: Write to GPPUDCLK0/1 to remove the clock
        clk_reg.write(0);
    }

    /// Configure SDIO pins for CYW43436 communication
    pub fn configure_sdio_pins(&self) {
        // Configure SDIO pins (GPIO 22-27) for EMMC2 Alt3 function
        self.set_function(22, GpioFunction::Alt3); // SDIO DAT0
        self.set_function(23, GpioFunction::Alt3); // SDIO DAT1
        self.set_function(24, GpioFunction::Alt3); // SDIO DAT2
        self.set_function(25, GpioFunction::Alt3); // SDIO DAT3
        self.set_function(26, GpioFunction::Alt3); // SDIO CLK
        self.set_function(27, GpioFunction::Alt3); // SDIO CMD

        // Set appropriate pull resistors for SDIO pins
        self.set_pull(22, GpioPull::PullUp); // DAT0 - pull up
        self.set_pull(23, GpioPull::PullUp); // DAT1 - pull up
        self.set_pull(24, GpioPull::PullUp); // DAT2 - pull up
        self.set_pull(25, GpioPull::PullUp); // DAT3 - pull up
        self.set_pull(26, GpioPull::None);   // CLK - no pull
        self.set_pull(27, GpioPull::PullUp); // CMD - pull up
    }

    /// Initialize CYW43436 power control pins
    pub fn init_cyw43436_power_pins(&self) {
        // Configure power enable pin (example: GPIO 2)
        self.set_function(2, GpioFunction::Output);
        self.set_low(2); // Start with power off
        
        // Configure reset pin (example: GPIO 3)
        self.set_function(3, GpioFunction::Output);
        self.set_low(3); // Start in reset state
    }

    /// Power on the CYW43436 chip
    pub fn power_on_cyw43436(&self) {
        // Release reset first
        self.set_high(3);
        
        // Small delay
        for _ in 0..1000 {
            unsafe { core::arch::asm!("nop") };
        }
        
        // Enable power
        self.set_high(2);
        
        // Wait for power to stabilize
        for _ in 0..10000 {
            unsafe { core::arch::asm!("nop") };
        }
    }

    /// Power off the CYW43436 chip
    pub fn power_off_cyw43436(&self) {
        // Disable power
        self.set_low(2);
        
        // Assert reset
        self.set_low(3);
    }

    /// Reset the CYW43436 chip
    pub fn reset_cyw43436(&self) {
        // Assert reset
        self.set_low(3);
        
        // Hold reset for sufficient time
        for _ in 0..50000 {
            unsafe { core::arch::asm!("nop") };
        }
        
        // Release reset
        self.set_high(3);
        
        // Wait for chip to come out of reset
        for _ in 0..10000 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}
