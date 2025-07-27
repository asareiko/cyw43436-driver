//! System timer for BCM2710
//! 
//! Provides timing functions for delays and timeouts

use super::mmio::Mmio;
use super::TIMER_BASE;

/// System timer counter lower 32 bits
const TIMER_CLO: u32 = 0x04;
/// System timer counter upper 32 bits  
const TIMER_CHI: u32 = 0x08;

/// System timer for BCM2710
pub struct SystemTimer {
    base: u32,
}

impl SystemTimer {
    /// Create a new system timer
    pub const fn new() -> Self {
        Self { base: TIMER_BASE }
    }

    /// Get the current timer value (microseconds since boot)
    pub fn get_time_us(&self) -> u64 {
        let clo_reg = unsafe { Mmio::<u32>::new(self.base + TIMER_CLO) };
        let chi_reg = unsafe { Mmio::<u32>::new(self.base + TIMER_CHI) };
        
        // Read high, then low, then high again to handle wraparound
        let high1 = chi_reg.read();
        let low = clo_reg.read();
        let high2 = chi_reg.read();
        
        // If high changed, re-read low
        let (high, low) = if high1 == high2 {
            (high1, low)
        } else {
            (high2, clo_reg.read())
        };
        
        ((high as u64) << 32) | (low as u64)
    }

    /// Delay for the specified number of microseconds
    pub fn delay_us(&self, us: u32) {
        let start = self.get_time_us();
        while self.get_time_us() - start < us as u64 {
            core::hint::spin_loop();
        }
    }

    /// Get the current timer value in milliseconds
    pub fn get_time_ms(&self) -> u32 {
        (self.get_time_us() / 1000) as u32
    }

    /// Delay for the specified number of milliseconds
    pub fn delay_ms(&self, ms: u32) {
        self.delay_us(ms * 1000);
    }

    /// Check if a timeout has occurred
    pub fn is_timeout(&self, start_time: u64, timeout_us: u32) -> bool {
        self.get_time_us() - start_time >= timeout_us as u64
    }
}
