//! SDIO controller for CYW43436 WiFi chip on BCM2710
//! 
//! This module implements the SDIO interface using the BCM2710's EMMC2 controller
//! to communicate with the CYW43436 WiFi chip.

use super::mmio::Mmio;
use super::timer::SystemTimer;
use super::EMMC2_BASE;
use bitflags::bitflags;
use core::mem;

/// EMMC2 register offsets for SDIO communication
const EMMC2_ARG2: u32 = 0x00;
const EMMC2_BLKSIZECNT: u32 = 0x04;
const EMMC2_ARG1: u32 = 0x08;
const EMMC2_CMDTM: u32 = 0x0C;
const EMMC2_RESP0: u32 = 0x10;
const EMMC2_RESP1: u32 = 0x14;
const EMMC2_RESP2: u32 = 0x18;
const EMMC2_RESP3: u32 = 0x1C;
const EMMC2_DATA: u32 = 0x20;
const EMMC2_STATUS: u32 = 0x24;
const EMMC2_CONTROL0: u32 = 0x28;
const EMMC2_CONTROL1: u32 = 0x2C;
const EMMC2_INTERRUPT: u32 = 0x30;
const EMMC2_IRPT_MASK: u32 = 0x34;
const EMMC2_IRPT_EN: u32 = 0x38;
const EMMC2_CONTROL2: u32 = 0x3C;
const EMMC2_FORCE_IRPT: u32 = 0x50;
const EMMC2_BOOT_TIMEOUT: u32 = 0x70;
const EMMC2_DBG_SEL: u32 = 0x74;
const EMMC2_EXRDFIFO_CFG: u32 = 0x80;
const EMMC2_EXRDFIFO_EN: u32 = 0x84;
const EMMC2_TUNE_STEP: u32 = 0x88;
const EMMC2_TUNE_STEPS_STD: u32 = 0x8C;
const EMMC2_TUNE_STEPS_DDR: u32 = 0x90;
const EMMC2_SPI_INT_SPT: u32 = 0xF0;
const EMMC2_SLOTISR_VER: u32 = 0xFC;

bitflags! {
    /// SDIO command flags
    pub struct SdioCommandFlags: u32 {
        /// Response expected
        const RESP_EXPECTED = 1 << 0;
        /// Long response (136-bit)
        const LONG_RESP = 1 << 1;
        /// Check response CRC
        const CHECK_RESP_CRC = 1 << 2;
        /// Check response index
        const CHECK_RESP_IDX = 1 << 3;
        /// Data present
        const DATA_PRESENT = 1 << 4;
        /// Multi block transfer
        const MULTI_BLOCK = 1 << 5;
        /// Read from card
        const READ_DATA = 1 << 6;
        /// Auto CMD12
        const AUTO_CMD12 = 1 << 7;
        /// Block count enable
        const BLOCK_CNT_EN = 1 << 8;
        /// DMA enable  
        const DMA_EN = 1 << 9;
    }
}

bitflags! {
    /// EMMC2 status register flags
    pub struct Emmc2Status: u32 {
        /// Command inhibit
        const CMD_INHIBIT = 1 << 0;
        /// Data inhibit
        const DATA_INHIBIT = 1 << 1;
        /// DAT line active
        const DAT_ACTIVE = 1 << 2;
        /// Write transfer active
        const WRITE_ACTIVE = 1 << 8;
        /// Read transfer active
        const READ_ACTIVE = 1 << 9;
        /// Buffer write enable
        const BUF_WR_EN = 1 << 10;
        /// Buffer read enable
        const BUF_RD_EN = 1 << 11;
        /// Card inserted
        const CARD_INSERTED = 1 << 16;
        /// Card state stable
        const CARD_STABLE = 1 << 17;
        /// Card detect pin level
        const CARD_DETECT = 1 << 18;
        /// Write protect switch pin
        const WRITE_PROTECT = 1 << 19;
        /// DAT[3:0] line signal level
        const DAT_LEVEL = 0xF << 20;
        /// CMD line signal level
        const CMD_LEVEL = 1 << 24;
    }
}

bitflags! {
    /// EMMC2 interrupt flags
    pub struct Emmc2Interrupt: u32 {
        /// Command complete
        const CMD_DONE = 1 << 0;
        /// Data transfer complete
        const DATA_DONE = 1 << 1;
        /// Block gap event
        const BLOCK_GAP = 1 << 2;
        /// Write ready
        const WRITE_RDY = 1 << 4;
        /// Read ready
        const READ_RDY = 1 << 5;
        /// Card insertion
        const CARD_INSERT = 1 << 6;
        /// Card removal
        const CARD_REMOVE = 1 << 7;
        /// Card interrupt
        const CARD = 1 << 8;
        /// Error occurred
        const ERR = 1 << 15;
        /// Command timeout error
        const CMD_TIMEOUT = 1 << 16;
        /// Command CRC error
        const CMD_CRC = 1 << 17;
        /// Command end bit error
        const CMD_END_BIT = 1 << 18;
        /// Command index error
        const CMD_INDEX = 1 << 19;
        /// Data timeout error
        const DATA_TIMEOUT = 1 << 20;
        /// Data CRC error
        const DATA_CRC = 1 << 21;
        /// Data end bit error
        const DATA_END_BIT = 1 << 22;
        /// Auto CMD12 error
        const ACMD12_ERR = 1 << 24;
    }
}

/// SDIO command structure
#[derive(Debug, Clone, Copy)]
pub struct SdioCommand {
    pub index: u8,
    pub arg: u32,
    pub flags: SdioCommandFlags,
    pub blocks: u16,
    pub block_size: u16,
}

/// SDIO response structure
#[derive(Debug, Clone, Copy)]
pub struct SdioResponse {
    pub data: [u32; 4],
}

/// SDIO controller errors
#[derive(Debug, Clone, Copy)]
pub enum SdioError {
    Timeout,
    CrcError,
    EndBitError,
    IndexError,
    DataTimeout,
    DataCrcError,
    DataEndBitError,
    AutoCmd12Error,
    InvalidCommand,
    CardNotPresent,
    BusyTimeout,
}

/// SDIO controller for CYW43436
pub struct SdioController {
    base: u32,
    timer: SystemTimer,
    clock_freq: u32,
}

impl SdioController {
    /// Create a new SDIO controller
    pub const fn new() -> Self {
        Self {
            base: EMMC2_BASE,
            timer: SystemTimer::new(),
            clock_freq: 0,
        }
    }

    /// Initialize the SDIO controller
    pub fn init(&mut self) -> Result<(), SdioError> {
        defmt::info!("Initializing SDIO controller...");

        // Reset the controller
        self.reset();

        // Wait for reset to complete
        self.timer.delay_ms(10);

        // Check if card is present (for SDIO, this might always be true)
        if !self.is_card_present() {
            defmt::warn!("SDIO card not detected");
            // For CYW43436, we might want to continue anyway as it's soldered on
        }

        // Set initial clock to low frequency for initialization (400kHz)
        self.set_clock(400_000)?;

        // Configure control registers
        self.configure_controller();

        // Enable interrupts
        self.enable_interrupts();

        defmt::info!("SDIO controller initialized successfully");
        Ok(())
    }

    /// Reset the SDIO controller
    fn reset(&self) {
        let control1 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL1) };
        
        // Software reset for all
        control1.write(0x07000000);
        
        // Wait for reset to complete
        let timeout_start = self.timer.get_time_us();
        while control1.read() & 0x07000000 != 0 {
            if self.timer.is_timeout(timeout_start, 100_000) {
                defmt::warn!("SDIO reset timeout");
                break;
            }
            core::hint::spin_loop();
        }
    }

    /// Check if SDIO card is present
    fn is_card_present(&self) -> bool {
        let status = unsafe { Mmio::<u32>::new(self.base + EMMC2_STATUS) };
        let status_val = Emmc2Status::from_bits_truncate(status.read());
        status_val.contains(Emmc2Status::CARD_INSERTED)
    }

    /// Set SDIO clock frequency
    fn set_clock(&mut self, freq_hz: u32) -> Result<(), SdioError> {
        let control1 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL1) };
        
        // Disable clock
        control1.modify(|val| val & !(1 << 2));
        
        // Wait for clock to stop
        let timeout_start = self.timer.get_time_us();
        while control1.read() & (1 << 3) != 0 {
            if self.timer.is_timeout(timeout_start, 100_000) {
                return Err(SdioError::Timeout);
            }
            core::hint::spin_loop();
        }

        // Calculate divisor for desired frequency
        // Base clock is typically 100MHz for BCM2710
        let base_clock = 100_000_000;
        let divisor = if freq_hz >= base_clock {
            1
        } else {
            (base_clock + freq_hz - 1) / freq_hz
        };

        // Set clock divisor
        let div_hi = (divisor >> 8) & 0x3;
        let div_lo = divisor & 0xFF;
        control1.modify(|val| (val & 0xFFFF003F) | ((div_hi << 6) | (div_lo << 8)));

        // Enable internal clock
        control1.modify(|val| val | (1 << 0));

        // Wait for clock to stabilize
        let timeout_start = self.timer.get_time_us();
        while control1.read() & (1 << 1) == 0 {
            if self.timer.is_timeout(timeout_start, 100_000) {
                return Err(SdioError::Timeout);
            }
            core::hint::spin_loop();
        }

        // Enable SD clock
        control1.modify(|val| val | (1 << 2));

        self.clock_freq = freq_hz;
        defmt::info!("SDIO clock set to {} Hz", freq_hz);
        Ok(())
    }

    /// Configure the SDIO controller
    fn configure_controller(&self) {
        let control0 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL0) };
        let control1 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL1) };
        let control2 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL2) };

        // Set bus width to 1-bit initially
        control0.modify(|val| val & !(1 << 1));

        // Set data timeout to maximum
        control1.modify(|val| (val & 0xFFF0FFFF) | (0xE << 16));

        // Configure voltage
        control0.modify(|val| val | (0x7 << 9)); // 3.3V
    }

    /// Enable SDIO interrupts
    fn enable_interrupts(&self) {
        let irpt_en = unsafe { Mmio::<u32>::new(self.base + EMMC2_IRPT_EN) };
        let irpt_mask = unsafe { Mmio::<u32>::new(self.base + EMMC2_IRPT_MASK) };

        // Enable basic interrupts
        let interrupts = Emmc2Interrupt::CMD_DONE 
            | Emmc2Interrupt::DATA_DONE
            | Emmc2Interrupt::BLOCK_GAP
            | Emmc2Interrupt::WRITE_RDY
            | Emmc2Interrupt::READ_RDY
            | Emmc2Interrupt::CARD
            | Emmc2Interrupt::ERR;

        irpt_en.write(interrupts.bits());
        irpt_mask.write(interrupts.bits());
    }

    /// Send SDIO command
    pub fn send_command(&self, cmd: SdioCommand) -> Result<SdioResponse, SdioError> {
        // Wait for command line to be ready
        self.wait_for_command_ready()?;

        // Set block size and count if data transfer
        if cmd.flags.contains(SdioCommandFlags::DATA_PRESENT) {
            let blksizecnt = unsafe { Mmio::<u32>::new(self.base + EMMC2_BLKSIZECNT) };
            let value = ((cmd.blocks as u32) << 16) | (cmd.block_size as u32);
            blksizecnt.write(value);
        }

        // Set argument
        let arg1 = unsafe { Mmio::<u32>::new(self.base + EMMC2_ARG1) };
        arg1.write(cmd.arg);

        // Prepare command register value
        let mut cmdtm_val = (cmd.index as u32) << 24;

        // Set response type
        if cmd.flags.contains(SdioCommandFlags::RESP_EXPECTED) {
            if cmd.flags.contains(SdioCommandFlags::LONG_RESP) {
                cmdtm_val |= 0x01 << 16; // Long response
            } else {
                cmdtm_val |= 0x02 << 16; // Short response
            }
        }

        // Set other flags
        if cmd.flags.contains(SdioCommandFlags::CHECK_RESP_CRC) {
            cmdtm_val |= 1 << 19;
        }
        if cmd.flags.contains(SdioCommandFlags::CHECK_RESP_IDX) {
            cmdtm_val |= 1 << 20;
        }
        if cmd.flags.contains(SdioCommandFlags::DATA_PRESENT) {
            cmdtm_val |= 1 << 21;
        }
        if cmd.flags.contains(SdioCommandFlags::MULTI_BLOCK) {
            cmdtm_val |= 1 << 5;
        }
        if cmd.flags.contains(SdioCommandFlags::READ_DATA) {
            cmdtm_val |= 1 << 4;
        }
        if cmd.flags.contains(SdioCommandFlags::AUTO_CMD12) {
            cmdtm_val |= 0x01 << 2;
        }
        if cmd.flags.contains(SdioCommandFlags::BLOCK_CNT_EN) {
            cmdtm_val |= 1 << 1;
        }

        // Send command
        let cmdtm = unsafe { Mmio::<u32>::new(self.base + EMMC2_CMDTM) };
        cmdtm.write(cmdtm_val);

        // Wait for command to complete
        self.wait_for_command_complete()?;

        // Read response if expected
        let mut response = SdioResponse { data: [0; 4] };
        if cmd.flags.contains(SdioCommandFlags::RESP_EXPECTED) {
            response = self.read_response(cmd.flags.contains(SdioCommandFlags::LONG_RESP))?;
        }

        Ok(response)
    }

    /// Wait for command line to be ready
    fn wait_for_command_ready(&self) -> Result<(), SdioError> {
        let status = unsafe { Mmio::<u32>::new(self.base + EMMC2_STATUS) };
        let timeout_start = self.timer.get_time_us();

        while status.read() & Emmc2Status::CMD_INHIBIT.bits() != 0 {
            if self.timer.is_timeout(timeout_start, 100_000) {
                return Err(SdioError::BusyTimeout);
            }
            core::hint::spin_loop();
        }

        Ok(())
    }

    /// Wait for command to complete
    fn wait_for_command_complete(&self) -> Result<(), SdioError> {
        let interrupt = unsafe { Mmio::<u32>::new(self.base + EMMC2_INTERRUPT) };
        let timeout_start = self.timer.get_time_us();

        loop {
            let int_status = Emmc2Interrupt::from_bits_truncate(interrupt.read());

            // Check for errors first
            if int_status.contains(Emmc2Interrupt::ERR) {
                // Clear interrupt
                interrupt.write(int_status.bits());
                
                if int_status.contains(Emmc2Interrupt::CMD_TIMEOUT) {
                    return Err(SdioError::Timeout);
                }
                if int_status.contains(Emmc2Interrupt::CMD_CRC) {
                    return Err(SdioError::CrcError);
                }
                if int_status.contains(Emmc2Interrupt::CMD_END_BIT) {
                    return Err(SdioError::EndBitError);
                }
                if int_status.contains(Emmc2Interrupt::CMD_INDEX) {
                    return Err(SdioError::IndexError);
                }
            }

            // Check for successful completion
            if int_status.contains(Emmc2Interrupt::CMD_DONE) {
                // Clear interrupt
                interrupt.write(Emmc2Interrupt::CMD_DONE.bits());
                return Ok(());
            }

            // Check for timeout
            if self.timer.is_timeout(timeout_start, 1_000_000) {
                return Err(SdioError::Timeout);
            }

            core::hint::spin_loop();
        }
    }

    /// Read command response
    fn read_response(&self, long_response: bool) -> Result<SdioResponse, SdioError> {
        let resp0 = unsafe { Mmio::<u32>::new(self.base + EMMC2_RESP0) };
        let resp1 = unsafe { Mmio::<u32>::new(self.base + EMMC2_RESP1) };
        let resp2 = unsafe { Mmio::<u32>::new(self.base + EMMC2_RESP2) };
        let resp3 = unsafe { Mmio::<u32>::new(self.base + EMMC2_RESP3) };

        let mut response = SdioResponse { data: [0; 4] };

        if long_response {
            // 136-bit response
            response.data[0] = resp0.read();
            response.data[1] = resp1.read();
            response.data[2] = resp2.read();
            response.data[3] = resp3.read();
        } else {
            // 48-bit response
            response.data[0] = resp0.read();
        }

        Ok(response)
    }

    /// Read data from SDIO
    pub fn read_data(&self, buffer: &mut [u8]) -> Result<(), SdioError> {
        let data_reg = unsafe { Mmio::<u32>::new(self.base + EMMC2_DATA) };
        let interrupt = unsafe { Mmio::<u32>::new(self.base + EMMC2_INTERRUPT) };
        
        let mut offset = 0;
        let timeout_start = self.timer.get_time_us();

        while offset < buffer.len() {
            // Wait for data ready or error
            loop {
                let int_status = Emmc2Interrupt::from_bits_truncate(interrupt.read());

                // Check for errors
                if int_status.contains(Emmc2Interrupt::ERR) {
                    interrupt.write(int_status.bits());
                    
                    if int_status.contains(Emmc2Interrupt::DATA_TIMEOUT) {
                        return Err(SdioError::DataTimeout);
                    }
                    if int_status.contains(Emmc2Interrupt::DATA_CRC) {
                        return Err(SdioError::DataCrcError);
                    }
                    if int_status.contains(Emmc2Interrupt::DATA_END_BIT) {
                        return Err(SdioError::DataEndBitError);
                    }
                }

                // Check if data is ready
                if int_status.contains(Emmc2Interrupt::READ_RDY) {
                    // Clear the interrupt
                    interrupt.write(Emmc2Interrupt::READ_RDY.bits());
                    break;
                }

                // Check for data transfer complete
                if int_status.contains(Emmc2Interrupt::DATA_DONE) {
                    interrupt.write(Emmc2Interrupt::DATA_DONE.bits());
                    return Ok(());
                }

                // Check for timeout
                if self.timer.is_timeout(timeout_start, 5_000_000) {
                    return Err(SdioError::DataTimeout);
                }

                core::hint::spin_loop();
            }

            // Read 32-bit words from FIFO
            let remaining = buffer.len() - offset;
            let words_to_read = (remaining + 3) / 4;

            for _ in 0..words_to_read.min(16) { // FIFO depth is typically 16 words
                if offset >= buffer.len() {
                    break;
                }

                let word = data_reg.read();
                let word_bytes = word.to_le_bytes();

                for i in 0..4 {
                    if offset < buffer.len() {
                        buffer[offset] = word_bytes[i];
                        offset += 1;
                    }
                }
            }
        }

        Ok(())
    }

    /// Write data to SDIO
    pub fn write_data(&self, buffer: &[u8]) -> Result<(), SdioError> {
        let data_reg = unsafe { Mmio::<u32>::new(self.base + EMMC2_DATA) };
        let interrupt = unsafe { Mmio::<u32>::new(self.base + EMMC2_INTERRUPT) };
        
        let mut offset = 0;
        let timeout_start = self.timer.get_time_us();

        while offset < buffer.len() {
            // Wait for write ready or error
            loop {
                let int_status = Emmc2Interrupt::from_bits_truncate(interrupt.read());

                // Check for errors
                if int_status.contains(Emmc2Interrupt::ERR) {
                    interrupt.write(int_status.bits());
                    
                    if int_status.contains(Emmc2Interrupt::DATA_TIMEOUT) {
                        return Err(SdioError::DataTimeout);
                    }
                    if int_status.contains(Emmc2Interrupt::DATA_CRC) {
                        return Err(SdioError::DataCrcError);
                    }
                    if int_status.contains(Emmc2Interrupt::DATA_END_BIT) {
                        return Err(SdioError::DataEndBitError);
                    }
                }

                // Check if ready for write
                if int_status.contains(Emmc2Interrupt::WRITE_RDY) {
                    // Clear the interrupt
                    interrupt.write(Emmc2Interrupt::WRITE_RDY.bits());
                    break;
                }

                // Check for data transfer complete
                if int_status.contains(Emmc2Interrupt::DATA_DONE) {
                    interrupt.write(Emmc2Interrupt::DATA_DONE.bits());
                    return Ok(());
                }

                // Check for timeout
                if self.timer.is_timeout(timeout_start, 5_000_000) {
                    return Err(SdioError::DataTimeout);
                }

                core::hint::spin_loop();
            }

            // Write 32-bit words to FIFO
            let remaining = buffer.len() - offset;
            let words_to_write = (remaining + 3) / 4;

            for _ in 0..words_to_write.min(16) { // FIFO depth is typically 16 words
                if offset >= buffer.len() {
                    break;
                }

                let mut word_bytes = [0u8; 4];
                for i in 0..4 {
                    if offset < buffer.len() {
                        word_bytes[i] = buffer[offset];
                        offset += 1;
                    }
                }

                let word = u32::from_le_bytes(word_bytes);
                data_reg.write(word);
            }
        }

        Ok(())
    }

    /// Set SDIO bus width
    pub fn set_bus_width(&self, width: u8) -> Result<(), SdioError> {
        let control0 = unsafe { Mmio::<u32>::new(self.base + EMMC2_CONTROL0) };

        match width {
            1 => {
                // 1-bit mode
                control0.modify(|val| val & !(1 << 1));
            }
            4 => {
                // 4-bit mode
                control0.modify(|val| val | (1 << 1));
            }
            _ => return Err(SdioError::InvalidCommand),
        }

        defmt::info!("SDIO bus width set to {} bits", width);
        Ok(())
    }

    /// Get current SDIO status
    pub fn get_status(&self) -> Emmc2Status {
        let status = unsafe { Mmio::<u32>::new(self.base + EMMC2_STATUS) };
        Emmc2Status::from_bits_truncate(status.read())
    }

    /// Get current interrupt status
    pub fn get_interrupts(&self) -> Emmc2Interrupt {
        let interrupt = unsafe { Mmio::<u32>::new(self.base + EMMC2_INTERRUPT) };
        Emmc2Interrupt::from_bits_truncate(interrupt.read())
    }

    /// Clear interrupt flags
    pub fn clear_interrupts(&self, flags: Emmc2Interrupt) {
        let interrupt = unsafe { Mmio::<u32>::new(self.base + EMMC2_INTERRUPT) };
        interrupt.write(flags.bits());
    }
}
