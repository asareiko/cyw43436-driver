//! Memory-Mapped I/O abstraction for safe hardware register access

use core::ptr::{read_volatile, write_volatile};

/// Memory-mapped I/O register wrapper for safe hardware access
pub struct Mmio<T> {
    addr: *mut T,
}

impl<T> Mmio<T> {
    /// Create a new MMIO register at the given address
    ///
    /// # Safety
    /// The caller must ensure the address is valid and points to a hardware register
    pub const unsafe fn new(addr: u32) -> Self {
        Self {
            addr: addr as *mut T,
        }
    }

    /// Read from the register
    pub fn read(&self) -> T
    where
        T: Copy,
    {
        unsafe { read_volatile(self.addr) }
    }

    /// Write to the register
    pub fn write(&self, value: T) {
        unsafe { write_volatile(self.addr, value) }
    }

    /// Modify the register using a closure
    pub fn modify<F>(&self, f: F)
    where
        T: Copy,
        F: FnOnce(T) -> T,
    {
        let value = self.read();
        self.write(f(value));
    }
}

unsafe impl<T> Send for Mmio<T> {}
unsafe impl<T> Sync for Mmio<T> {}
