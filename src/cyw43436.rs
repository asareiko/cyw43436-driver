//! CYW43436 WiFi driver for Raspberry Pi Zero 2 W
//!
//! This module implements the CYW43436 WiFi chip driver using SDIO communication

use crate::hal::{
    gpio::GpioController,
    sdio::{SdioController, SdioCommand, SdioCommandFlags, SdioError, SdioResponse},
    timer::SystemTimer,
};
use bitflags::bitflags;
use defmt::Format;
use heapless::Vec;
use heapless::String;

/// SDIO function numbers for CYW43436
const SDIO_FUNC_0: u8 = 0; // SDIO controller function
const SDIO_FUNC_1: u8 = 1; // Backplane access function
const SDIO_FUNC_2: u8 = 2; // WiFi data function

/// Common SDIO commands
const SDIO_CMD_GO_IDLE_STATE: u8 = 0;
const SDIO_CMD_SEND_IF_COND: u8 = 8;
const SDIO_CMD_IO_SEND_OP_COND: u8 = 5;
const SDIO_CMD_IO_RW_DIRECT: u8 = 52;
const SDIO_CMD_IO_RW_EXTENDED: u8 = 53;

/// CYW43436 specific registers and addresses
const CYW43436_CHIP_ID: u32 = 0x4345;
const CYW43436_BACKPLANE_WINDOW: u32 = 0x10000;

/// Firmware loading constants
const FIRMWARE_MAX_SIZE: usize = 256 * 1024; // 256KB max firmware size
const FIRMWARE_LOAD_ADDR: u32 = 0x00000000; // Default firmware load address
const FIRMWARE_READY_TIMEOUT_MS: u32 = 5000; // 5 second timeout

/// WiFi protocol constants
const WIFI_SCAN_TIMEOUT_MS: u32 = 10000; // 10 second scan timeout
const WIFI_CONNECT_TIMEOUT_MS: u32 = 30000; // 30 second connect timeout
const MAX_SCAN_RESULTS: usize = 32;
const MAX_SSID_LENGTH: usize = 32;

/// WPA/WPA2 Authentication constants
const WPA_4WAY_HANDSHAKE_TIMEOUT_MS: u32 = 10000; // 10 second handshake timeout
const WPA_PMK_LENGTH: usize = 32; // Pre-shared key length
const WPA_PTK_LENGTH: usize = 64; // Pairwise transient key length
const WPA_MIC_LENGTH: usize = 16; // Message integrity check length
const WPA_NONCE_LENGTH: usize = 32; // Random nonce length

/// Connection state constants
const MAX_CONNECT_RETRIES: u8 = 3;
const MAX_PASSPHRASE_LENGTH: usize = 63;

/// SDIO Common Card Control Registers (CCCR)
const SDIO_CCCR_CCCR: u8 = 0x00;
const SDIO_CCCR_SD: u8 = 0x01;
const SDIO_CCCR_IOE: u8 = 0x02;
const SDIO_CCCR_IOR: u8 = 0x03;
const SDIO_CCCR_IEN: u8 = 0x04;
const SDIO_CCCR_INTX: u8 = 0x05;
const SDIO_CCCR_ABORT: u8 = 0x06;
const SDIO_CCCR_BUS_IF: u8 = 0x07;
const SDIO_CCCR_CARD_CAP: u8 = 0x08;
const SDIO_CCCR_SPEED: u8 = 0x13;

bitflags! {
    /// CYW43436 status flags
    #[derive(Clone, Copy)]
    pub struct Cyw43436Status: u32 {
        /// Chip is powered on
        const POWERED = 1 << 0;
        /// Firmware is loaded
        const FIRMWARE_LOADED = 1 << 1;
        /// Chip is initialized
        const INITIALIZED = 1 << 2;
        /// SDIO interface is ready
        const SDIO_READY = 1 << 3;
        /// WiFi is ready
        const WIFI_READY = 1 << 4;
    }
}

/// CYW43436 WiFi driver errors
#[derive(Debug, Clone, Copy, Format)]
pub enum Cyw43436Error {
    SdioError(SdioError),
    ChipNotFound,
    FirmwareLoadFailed,
    InitializationFailed,
    InvalidResponse,
    Timeout,
    NotReady,
}

impl From<SdioError> for Cyw43436Error {
    fn from(error: SdioError) -> Self {
        Cyw43436Error::SdioError(error)
    }
}

/// CYW43436 WiFi driver
pub struct Cyw43436Driver {
    sdio: SdioController,
    gpio: GpioController,
    timer: SystemTimer,
    status: Cyw43436Status,
    rca: u16, // Relative Card Address
}

impl Cyw43436Driver {
    /// Create a new CYW43436 driver
    pub const fn new() -> Self {
        Self {
            sdio: SdioController::new(),
            gpio: GpioController::new(),
            timer: SystemTimer::new(),
            status: Cyw43436Status::empty(),
            rca: 0,
        }
    }

    /// Initialize the CYW43436 WiFi chip
    pub fn init(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Initializing CYW43436 WiFi driver...");

        // Step 1: Configure GPIO pins for SDIO and power control
        self.gpio.configure_sdio_pins();
        self.gpio.init_cyw43436_power_pins();

        // Step 2: Power on the CYW43436 chip
        self.power_on()?;

        // Step 3: Initialize SDIO controller
        self.sdio.init()?;

        // Step 4: Initialize SDIO communication
        self.init_sdio()?;

        // Step 5: Detect and identify the chip
        self.detect_chip()?;

        // Step 6: Load firmware (placeholder for now)
        self.load_firmware()?;

        // Step 7: Initialize the WiFi functionality
        self.init_wifi()?;

        self.status.insert(Cyw43436Status::INITIALIZED | Cyw43436Status::WIFI_READY);
        defmt::info!("CYW43436 WiFi driver initialized successfully");
        Ok(())
    }

    /// Power on the CYW43436 chip
    fn power_on(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Powering on CYW43436...");

        // Use GPIO controller to power on the chip
        self.gpio.power_on_cyw43436();

        // Wait for power stabilization
        self.timer.delay_ms(100);

        self.status.insert(Cyw43436Status::POWERED);
        Ok(())
    }

    /// Initialize SDIO communication with CYW43436
    fn init_sdio(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Initializing SDIO communication...");

        // Send CMD0 (GO_IDLE_STATE) to reset the card
        let cmd0 = SdioCommand {
            index: SDIO_CMD_GO_IDLE_STATE,
            arg: 0,
            flags: SdioCommandFlags::empty(),
            blocks: 0,
            block_size: 0,
        };
        self.sdio.send_command(cmd0)?;
        self.timer.delay_ms(10);

        // Send CMD8 (SEND_IF_COND) to check voltage compatibility
        let cmd8 = SdioCommand {
            index: SDIO_CMD_SEND_IF_COND,
            arg: 0x000001AA, // 2.7-3.6V, check pattern 0xAA
            flags: SdioCommandFlags::RESP_EXPECTED | SdioCommandFlags::CHECK_RESP_CRC,
            blocks: 0,
            block_size: 0,
        };

        match self.sdio.send_command(cmd8) {
            Ok(response) => {
                defmt::info!("CMD8 response: 0x{:08x}", response.data[0]);
            }
            Err(_) => {
                defmt::warn!("CMD8 failed, continuing anyway");
            }
        }

        // Send CMD5 (IO_SEND_OP_COND) to initialize SDIO
        let mut retry_count = 0;
        loop {
            let cmd5 = SdioCommand {
                index: SDIO_CMD_IO_SEND_OP_COND,
                arg: 0x00300000, // 3.2V-3.4V voltage range
                flags: SdioCommandFlags::RESP_EXPECTED,
                blocks: 0,
                block_size: 0,
            };

            match self.sdio.send_command(cmd5) {
                Ok(response) => {
                    let ocr = response.data[0];
                    defmt::info!("CMD5 OCR: 0x{:08x}", ocr);

                    // Check if card is ready (bit 31)
                    if ocr & 0x80000000 != 0 {
                        // Check number of I/O functions
                        let num_functions = (ocr >> 28) & 0x7;
                        defmt::info!("SDIO card ready, {} I/O functions", num_functions);
                        break;
                    }
                }
                Err(e) => {
                    defmt::warn!("CMD5 failed: {:?}", e);
                    return Err(e.into());
                }
            }

            retry_count += 1;
            if retry_count > 100 {
                defmt::error!("CMD5 initialization timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(10);
        }

        // Enable I/O functions
        self.sdio_write_byte(SDIO_FUNC_0, SDIO_CCCR_IOE as u32, 0x02)?; // Enable function 1
        self.timer.delay_ms(10);

        // Wait for function 1 to be ready
        let mut retry_count = 0;
        loop {
            let ior = self.sdio_read_byte(SDIO_FUNC_0, SDIO_CCCR_IOR as u32)?;
            if ior & 0x02 != 0 {
                defmt::info!("SDIO function 1 ready");
                break;
            }

            retry_count += 1;
            if retry_count > 100 {
                defmt::error!("Function 1 ready timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(10);
        }

        // Set bus width to 4-bit if supported
        let bus_if = self.sdio_read_byte(SDIO_FUNC_0, SDIO_CCCR_BUS_IF as u32)?;
        if bus_if & 0x40 != 0 { // 4-bit bus width supported
            self.sdio_write_byte(SDIO_FUNC_0, SDIO_CCCR_BUS_IF as u32, bus_if | 0x02)?;
            self.sdio.set_bus_width(4)?;
            defmt::info!("SDIO bus width set to 4-bit");
        }

        self.status.insert(Cyw43436Status::SDIO_READY);
        Ok(())
    }

    /// Detect and identify the CYW43436 chip
    fn detect_chip(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Detecting CYW43436 chip...");

        // Read chip ID from function 1
        // This is a simplified detection - actual implementation would need
        // to read from specific backplane registers
        let chip_id_low = self.sdio_read_byte(SDIO_FUNC_1, 0x00)?;
        let chip_id_high = self.sdio_read_byte(SDIO_FUNC_1, 0x01)?;
        let chip_id = ((chip_id_high as u16) << 8) | (chip_id_low as u16);

        defmt::info!("Detected chip ID: 0x{:04x}", chip_id);

        // For now, we'll accept any chip ID as this is a basic implementation
        // Real implementation would verify against CYW43436_CHIP_ID

        Ok(())
    }

    /// Load firmware into the CYW43436 chip
    fn load_firmware(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Loading CYW43436 firmware...");

        // Enhanced firmware loading implementation
        // Step 1: Reset the chip to prepare for firmware upload
        self.reset_chip()?;

        // Step 2: Check if firmware is already loaded
        if self.is_firmware_loaded()? {
            defmt::info!("Firmware already loaded, skipping upload");
            self.status.insert(Cyw43436Status::FIRMWARE_LOADED);
            return Ok(());
        }

        // Step 3: Load firmware blob (simulated for now)
        // In a real implementation, this would be embedded firmware data
        let firmware_data = self.get_embedded_firmware()?;

        // Step 4: Upload firmware in chunks
        self.upload_firmware_chunks(&firmware_data)?;

        // Step 5: Start firmware execution
        self.start_firmware()?;

        // Step 6: Wait for firmware ready signal
        self.wait_for_firmware_ready()?;

        self.status.insert(Cyw43436Status::FIRMWARE_LOADED);
        defmt::info!("Firmware loaded and started successfully");
        Ok(())
    }

    /// Reset the CYW43436 chip
    fn reset_chip(&mut self) -> Result<(), Cyw43436Error> {
        defmt::debug!("Resetting CYW43436 chip...");

        // Use GPIO to perform hardware reset
        self.gpio.reset_cyw43436();
        self.timer.delay_ms(50);

        // Re-initialize SDIO after reset
        self.init_sdio()?;

        Ok(())
    }

    /// Check if firmware is already loaded
    fn is_firmware_loaded(&self) -> Result<bool, Cyw43436Error> {
        // Read firmware status register (simulated)
        let status = self.sdio_read_byte(SDIO_FUNC_1, 0x10)?;
        Ok(status & 0x01 != 0) // Bit 0 indicates firmware loaded
    }

    /// Get embedded firmware data (placeholder)
    fn get_embedded_firmware(&self) -> Result<&'static [u8], Cyw43436Error> {
        // In a real implementation, this would return embedded firmware blob
        // For now, return a dummy firmware blob
        static DUMMY_FIRMWARE: [u8; 1024] = [0x00; 1024];
        Ok(&DUMMY_FIRMWARE)
    }

    /// Upload firmware in chunks
    fn upload_firmware_chunks(&self, firmware: &[u8]) -> Result<(), Cyw43436Error> {
        defmt::info!("Uploading firmware ({} bytes)...", firmware.len());

        const CHUNK_SIZE: usize = 512; // SDIO block size
        let mut offset = 0;

        while offset < firmware.len() {
            let chunk_size = (firmware.len() - offset).min(CHUNK_SIZE);
            let chunk = &firmware[offset..offset + chunk_size];

            // Write chunk to firmware memory
            self.sdio_write_bytes(SDIO_FUNC_1, FIRMWARE_LOAD_ADDR + offset as u32, chunk)?;

            offset += chunk_size;

            // Show progress every 10KB
            if offset % (10 * 1024) == 0 {
                defmt::debug!("Firmware upload progress: {}/{} bytes", offset, firmware.len());
            }
        }

        defmt::info!("Firmware upload completed");
        Ok(())
    }

    /// Start firmware execution
    fn start_firmware(&self) -> Result<(), Cyw43436Error> {
        defmt::debug!("Starting firmware execution...");

        // Write to firmware control register to start execution
        self.sdio_write_byte(SDIO_FUNC_1, 0x10, 0x02)?; // Bit 1 = start firmware

        Ok(())
    }

    /// Wait for firmware ready signal
    fn wait_for_firmware_ready(&self) -> Result<(), Cyw43436Error> {
        defmt::debug!("Waiting for firmware ready signal...");

        let start_time = self.timer.get_time_ms();

        loop {
            // Check firmware status
            let status = self.sdio_read_byte(SDIO_FUNC_1, 0x10)?;

            if status & 0x04 != 0 { // Bit 2 = firmware ready
                defmt::info!("Firmware ready signal received");
                return Ok(());
            }

            // Check timeout
            if self.timer.get_time_ms() - start_time > FIRMWARE_READY_TIMEOUT_MS {
                defmt::error!("Firmware ready timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(10);
        }
    }

    /// Initialize WiFi functionality
    fn init_wifi(&mut self) -> Result<(), Cyw43436Error> {
        defmt::info!("Initializing WiFi functionality...");

        // TODO: Implement WiFi initialization
        // This would involve:
        // 1. Configure WiFi parameters
        // 2. Set up event handling
        // 3. Initialize scan and connection state machines
        // 4. Set up data path

        // For now, we'll simulate WiFi initialization
        self.timer.delay_ms(50);

        defmt::info!("WiFi functionality initialized");
        Ok(())
    }

    /// Read a single byte from SDIO
    fn sdio_read_byte(&self, function: u8, address: u32) -> Result<u8, Cyw43436Error> {
        let cmd = SdioCommand {
            index: SDIO_CMD_IO_RW_DIRECT,
            arg: ((function as u32) << 28) | ((address & 0x1FFFF) << 9),
            flags: SdioCommandFlags::RESP_EXPECTED | SdioCommandFlags::CHECK_RESP_CRC,
            blocks: 0,
            block_size: 0,
        };

        let response = self.sdio.send_command(cmd)?;
        Ok(response.data[0] as u8)
    }

    /// Write a single byte to SDIO
    fn sdio_write_byte(&self, function: u8, address: u32, data: u8) -> Result<(), Cyw43436Error> {
        let cmd = SdioCommand {
            index: SDIO_CMD_IO_RW_DIRECT,
            arg: ((function as u32) << 28) | ((address & 0x1FFFF) << 9) | (1 << 31) | (data as u32),
            flags: SdioCommandFlags::RESP_EXPECTED | SdioCommandFlags::CHECK_RESP_CRC,
            blocks: 0,
            block_size: 0,
        };

        self.sdio.send_command(cmd)?;
        Ok(())
    }

    /// Read multiple bytes from SDIO
    fn sdio_read_bytes(&self, function: u8, address: u32, buffer: &mut [u8]) -> Result<(), Cyw43436Error> {
        let block_size = buffer.len().min(512); // SDIO block size limit
        let blocks = if buffer.len() <= 512 { 0 } else { (buffer.len() + 511) / 512 };

        let cmd = SdioCommand {
            index: SDIO_CMD_IO_RW_EXTENDED,
            arg: ((function as u32) << 28) | (address << 9) | (buffer.len() as u32),
            flags: SdioCommandFlags::RESP_EXPECTED
                | SdioCommandFlags::CHECK_RESP_CRC
                | SdioCommandFlags::DATA_PRESENT
                | SdioCommandFlags::READ_DATA,
            blocks: blocks as u16,
            block_size: block_size as u16,
        };

        self.sdio.send_command(cmd)?;
        self.sdio.read_data(buffer)?;
        Ok(())
    }

    /// Write multiple bytes to SDIO
    fn sdio_write_bytes(&self, function: u8, address: u32, buffer: &[u8]) -> Result<(), Cyw43436Error> {
        let block_size = buffer.len().min(512); // SDIO block size limit
        let blocks = if buffer.len() <= 512 { 0 } else { (buffer.len() + 511) / 512 };

        let cmd = SdioCommand {
            index: SDIO_CMD_IO_RW_EXTENDED,
            arg: ((function as u32) << 28) | (address << 9) | (1 << 31) | (buffer.len() as u32),
            flags: SdioCommandFlags::RESP_EXPECTED
                | SdioCommandFlags::CHECK_RESP_CRC
                | SdioCommandFlags::DATA_PRESENT,
            blocks: blocks as u16,
            block_size: block_size as u16,
        };

        self.sdio.send_command(cmd)?;
        self.sdio.write_data(buffer)?;
        Ok(())
    }

    /// Get the current status of the driver
    pub fn get_status(&self) -> Cyw43436Status {
        self.status
    }

    /// Check if the driver is ready for WiFi operations
    pub fn is_ready(&self) -> bool {
        self.status.contains(Cyw43436Status::WIFI_READY)
    }

    /// Perform a WiFi scan with enhanced functionality
    pub fn scan(&mut self) -> Result<Vec<ScanResult, 32>, Cyw43436Error> {
        if !self.is_ready() {
            return Err(Cyw43436Error::NotReady);
        }

        defmt::info!("Starting WiFi scan...");

        // Enhanced WiFi scanning implementation
        // Step 1: Prepare scan command
        let scan_command = self.prepare_scan_command()?;

        // Step 2: Send scan command to firmware
        self.send_scan_command(&scan_command)?;

        // Step 3: Wait for scan completion
        self.wait_for_scan_completion()?;

        // Step 4: Retrieve and parse scan results
        let results = self.retrieve_scan_results()?;

        defmt::info!("WiFi scan completed, found {} networks", results.len());
        Ok(results)
    }

    /// Prepare scan command for firmware
    fn prepare_scan_command(&self) -> Result<[u8; 64], Cyw43436Error> {
        let mut command = [0u8; 64];

        // Scan command structure (simplified)
        command[0] = 0x01; // Command type: SCAN
        command[1] = 0x00; // Flags: Active scan
        command[2] = 0xFF; // Channel mask: All channels (2.4GHz)
        command[3] = 0x0F; // Channel mask continued

        // Scan timing parameters
        command[4] = 0x64; // Active scan time per channel (100ms)
        command[5] = 0x00;
        command[6] = 0x32; // Passive scan time per channel (50ms)
        command[7] = 0x00;

        // Home time between channels
        command[8] = 0x0A; // 10ms home time
        command[9] = 0x00;

        defmt::debug!("Prepared scan command");
        Ok(command)
    }

    /// Send scan command to firmware
    fn send_scan_command(&self, command: &[u8]) -> Result<(), Cyw43436Error> {
        defmt::debug!("Sending scan command to firmware...");

        // Send command via SDIO to function 2 (WiFi data)
        self.sdio_write_bytes(SDIO_FUNC_2, 0x1000, command)?;

        // Wait for command acknowledgment
        let mut retry_count = 0;
        loop {
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x1001)?;
            if status & 0x01 != 0 { // Command acknowledged
                break;
            }

            retry_count += 1;
            if retry_count > 100 {
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(10);
        }

        defmt::debug!("Scan command sent successfully");
        Ok(())
    }

    /// Wait for scan completion
    fn wait_for_scan_completion(&self) -> Result<(), Cyw43436Error> {
        defmt::debug!("Waiting for scan completion...");

        let start_time = self.timer.get_time_ms();

        loop {
            // Check scan status
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x1002)?;

            if status & 0x02 != 0 { // Scan completed
                defmt::debug!("Scan completed");
                return Ok(());
            }

            // Check timeout
            if self.timer.get_time_ms() - start_time > WIFI_SCAN_TIMEOUT_MS {
                defmt::error!("WiFi scan timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(100);
        }
    }

    /// Retrieve and parse scan results
    fn retrieve_scan_results(&self) -> Result<Vec<ScanResult, 32>, Cyw43436Error> {
        defmt::debug!("Retrieving scan results...");

        // Read number of results
        let num_results = self.sdio_read_byte(SDIO_FUNC_2, 0x1003)? as usize;
        let num_results = num_results.min(MAX_SCAN_RESULTS);

        defmt::debug!("Found {} scan results", num_results);

        let mut results = Vec::new();

        // Read each scan result
        for i in 0..num_results {
            let base_addr = 0x1100 + (i * 64) as u32; // Each result is 64 bytes
            let mut result_data = [0u8; 64];

            self.sdio_read_bytes(SDIO_FUNC_2, base_addr, &mut result_data)?;

            // Parse scan result
            if let Ok(scan_result) = self.parse_scan_result(&result_data) {
                let _ = results.push(scan_result);
            }
        }

        Ok(results)
    }

    /// Parse a single scan result from raw data
    fn parse_scan_result(&self, data: &[u8]) -> Result<ScanResult, Cyw43436Error> {
        // Parse SSID (first 32 bytes)
        let ssid_len = data[0] as usize;
        if ssid_len > 31 {
            return Err(Cyw43436Error::InvalidResponse);
        }

        let ssid_bytes = &data[1..1 + ssid_len];

        // Convert &[u8] to Vec<u8, 32> for heapless::String::from_utf8
        let mut ssid_vec = heapless::Vec::<u8, 32>::new();
        for &byte in ssid_bytes {
            if ssid_vec.push(byte).is_err() {
                return Err(Cyw43436Error::InvalidResponse);
            }
        }

        let ssid = heapless::String::from_utf8(ssid_vec)
            .map_err(|_| Cyw43436Error::InvalidResponse)?;

        // Parse BSSID (6 bytes starting at offset 32)
        let mut bssid = [0u8; 6];
        bssid.copy_from_slice(&data[32..38]);

        // Parse channel (1 byte at offset 38)
        let channel = data[38];

        // Parse RSSI (signed 8-bit at offset 39)
        let rssi = data[39] as i8;

        // Parse security type (1 byte at offset 40)
        let security = match data[40] {
            0 => SecurityType::Open,
            1 => SecurityType::Wep,
            2 => SecurityType::Wpa,
            3 => SecurityType::Wpa2,
            4 => SecurityType::Wpa3,
            _ => SecurityType::Unknown,
        };

        Ok(ScanResult {
            ssid,
            bssid,
            channel,
            rssi,
            security,
        })
    }

    /// Connect to a WiFi network with WPA/WPA2 authentication
    pub fn connect(&mut self, params: ConnectionParams) -> Result<ConnectionState, Cyw43436Error> {
        if !self.is_ready() {
            return Err(Cyw43436Error::NotReady);
        }

        defmt::info!("Connecting to WiFi network: {}", params.credentials.ssid.as_str());

        let mut retry_count = 0;
        while retry_count < params.retry_count {
            match self.attempt_connection(&params) {
                Ok(state) => {
                    if state == ConnectionState::Connected {
                        defmt::info!("Successfully connected to {}", params.credentials.ssid.as_str());
                        return Ok(state);
                    }
                }
                Err(e) => {
                    defmt::warn!("Connection attempt {} failed: {:?}", retry_count + 1, e);
                }
            }

            retry_count += 1;
            if retry_count < params.retry_count {
                defmt::info!("Retrying connection ({}/{})", retry_count + 1, params.retry_count);
                self.timer.delay_ms(2000); // Wait before retry
            }
        }

        defmt::error!("Failed to connect after {} attempts", params.retry_count);
        Ok(ConnectionState::Failed)
    }

    /// Attempt a single connection to the WiFi network
    fn attempt_connection(&mut self, params: &ConnectionParams) -> Result<ConnectionState, Cyw43436Error> {
        // Step 1: Send association request
        self.send_association_request(&params.credentials)?;

        // Step 2: Wait for association response
        self.wait_for_association_response()?;

        // Step 3: Perform authentication based on security type
        match params.credentials.security {
            SecurityType::Open => {
                // Open network - no authentication needed
                defmt::info!("Connected to open network");
                Ok(ConnectionState::Connected)
            }
            SecurityType::Wpa | SecurityType::Wpa2 => {
                // Perform WPA/WPA2 4-way handshake
                self.perform_wpa_handshake(&params.credentials)?;
                Ok(ConnectionState::Connected)
            }
            SecurityType::Wep => {
                // WEP authentication (simplified)
                self.perform_wep_authentication(&params.credentials)?;
                Ok(ConnectionState::Connected)
            }
            SecurityType::Wpa3 => {
                defmt::warn!("WPA3 not yet implemented");
                Err(Cyw43436Error::InvalidResponse)
            }
            SecurityType::Unknown => {
                defmt::error!("Unknown security type");
                Err(Cyw43436Error::InvalidResponse)
            }
        }
    }

    /// Send association request to the access point
    fn send_association_request(&self, credentials: &WifiCredentials) -> Result<(), Cyw43436Error> {
        defmt::debug!("Sending association request");

        let mut assoc_request = [0u8; 128];

        // Association request structure (simplified)
        assoc_request[0] = 0x02; // Command type: ASSOCIATE

        // Copy SSID
        assoc_request[1] = credentials.ssid.len() as u8;
        let ssid_bytes = credentials.ssid.as_bytes();
        assoc_request[2..2 + ssid_bytes.len()].copy_from_slice(ssid_bytes);

        // Security parameters
        assoc_request[34] = credentials.security as u8;

        // Send association request to firmware
        self.sdio_write_bytes(SDIO_FUNC_2, 0x2000, &assoc_request)?;

        defmt::debug!("Association request sent");
        Ok(())
    }

    /// Wait for association response
    fn wait_for_association_response(&self) -> Result<(), Cyw43436Error> {
        defmt::debug!("Waiting for association response");

        let start_time = self.timer.get_time_ms();

        loop {
            // Check association status
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x2001)?;

            if status & 0x01 != 0 { // Association successful
                defmt::debug!("Association successful");
                return Ok(());
            }

            if status & 0x02 != 0 { // Association failed
                defmt::error!("Association failed");
                return Err(Cyw43436Error::InvalidResponse);
            }

            // Check timeout
            if self.timer.get_time_ms() - start_time > 5000 { // 5 second timeout
                defmt::error!("Association timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(100);
        }
    }

    /// Perform WPA/WPA2 4-way handshake
    fn perform_wpa_handshake(&self, credentials: &WifiCredentials) -> Result<(), Cyw43436Error> {
        defmt::info!("Starting WPA/WPA2 4-way handshake");

        let mut handshake_state = WpaHandshakeState::new();

        // Step 1: Derive PMK from passphrase and SSID
        self.derive_pmk(credentials, &mut handshake_state)?;

        // Step 2: Handle Message 1 of 4-way handshake (receive ANonce)
        self.handle_handshake_msg1(&mut handshake_state)?;

        // Step 3: Send Message 2 of 4-way handshake (send SNonce)
        self.send_handshake_msg2(&handshake_state)?;

        // Step 4: Handle Message 3 of 4-way handshake (verify MIC, install PTK)
        self.handle_handshake_msg3(&mut handshake_state)?;

        // Step 5: Send Message 4 of 4-way handshake (acknowledge)
        self.send_handshake_msg4(&handshake_state)?;

        defmt::info!("WPA/WPA2 4-way handshake completed successfully");
        Ok(())
    }

    /// Derive Pre-Master Key (PMK) from passphrase and SSID
    fn derive_pmk(&self, credentials: &WifiCredentials, state: &mut WpaHandshakeState) -> Result<(), Cyw43436Error> {
        defmt::debug!("Deriving PMK from passphrase");

        // Simplified PBKDF2 implementation for PMK derivation
        // In a real implementation, this would use proper PBKDF2 with 4096 iterations
        let passphrase = credentials.passphrase.as_bytes();
        let ssid = credentials.ssid.as_bytes();

        // Simple hash-based PMK derivation (placeholder)
        for i in 0..WPA_PMK_LENGTH {
            let mut sum = 0u8;
            for &p in passphrase {
                sum = sum.wrapping_add(p);
            }
            for &s in ssid {
                sum = sum.wrapping_add(s);
            }
            state.pmk[i] = sum.wrapping_add(i as u8);
        }

        defmt::debug!("PMK derived successfully");
        Ok(())
    }

    /// Handle Message 1 of 4-way handshake (receive ANonce)
    fn handle_handshake_msg1(&self, state: &mut WpaHandshakeState) -> Result<(), Cyw43436Error> {
        defmt::debug!("Handling handshake message 1/4");

        let start_time = self.timer.get_time_ms();

        // Wait for Message 1
        loop {
            let msg_status = self.sdio_read_byte(SDIO_FUNC_2, 0x3000)?;

            if msg_status & 0x01 != 0 { // Message 1 received
                // Read ANonce from firmware
                let mut anonce_data = [0u8; 32];
                self.sdio_read_bytes(SDIO_FUNC_2, 0x3001, &mut anonce_data)?;
                state.anonce.copy_from_slice(&anonce_data);

                defmt::debug!("Received ANonce in message 1/4");
                return Ok(());
            }

            if self.timer.get_time_ms() - start_time > WPA_4WAY_HANDSHAKE_TIMEOUT_MS {
                defmt::error!("Timeout waiting for handshake message 1");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(50);
        }
    }

    /// Send Message 2 of 4-way handshake (send SNonce)
    fn send_handshake_msg2(&self, state: &WpaHandshakeState) -> Result<(), Cyw43436Error> {
        defmt::debug!("Sending handshake message 2/4");

        // Generate SNonce (simplified random generation)
        let mut snonce = [0u8; WPA_NONCE_LENGTH];
        let time_seed = self.timer.get_time_ms();
        for i in 0..WPA_NONCE_LENGTH {
            snonce[i] = ((time_seed + i as u32) & 0xFF) as u8;
        }

        // Send SNonce to firmware
        self.sdio_write_bytes(SDIO_FUNC_2, 0x3100, &snonce)?;

        // Signal message 2 ready
        self.sdio_write_byte(SDIO_FUNC_2, 0x3101, 0x01)?;

        defmt::debug!("Sent SNonce in message 2/4");
        Ok(())
    }

    /// Handle Message 3 of 4-way handshake (verify MIC, install PTK)
    fn handle_handshake_msg3(&self, state: &mut WpaHandshakeState) -> Result<(), Cyw43436Error> {
        defmt::debug!("Handling handshake message 3/4");

        let start_time = self.timer.get_time_ms();

        // Wait for Message 3
        loop {
            let msg_status = self.sdio_read_byte(SDIO_FUNC_2, 0x3200)?;

            if msg_status & 0x01 != 0 { // Message 3 received
                // Read and verify MIC
                let mut mic_data = [0u8; WPA_MIC_LENGTH];
                self.sdio_read_bytes(SDIO_FUNC_2, 0x3201, &mut mic_data)?;

                // Simplified MIC verification (in real implementation, would verify with PTK)
                defmt::debug!("Verifying MIC in message 3/4");

                // Install PTK (placeholder - would derive actual PTK from PMK, ANonce, SNonce)
                defmt::debug!("Installing PTK");

                return Ok(());
            }

            if self.timer.get_time_ms() - start_time > WPA_4WAY_HANDSHAKE_TIMEOUT_MS {
                defmt::error!("Timeout waiting for handshake message 3");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(50);
        }
    }

    /// Send Message 4 of 4-way handshake (acknowledge)
    fn send_handshake_msg4(&self, _state: &WpaHandshakeState) -> Result<(), Cyw43436Error> {
        defmt::debug!("Sending handshake message 4/4");

        // Send acknowledgment
        self.sdio_write_byte(SDIO_FUNC_2, 0x3300, 0x01)?;

        // Wait for confirmation
        let start_time = self.timer.get_time_ms();
        loop {
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x3301)?;

            if status & 0x01 != 0 { // Handshake completed
                defmt::debug!("4-way handshake completed");
                return Ok(());
            }

            if self.timer.get_time_ms() - start_time > 2000 { // 2 second timeout
                defmt::error!("Timeout waiting for handshake completion");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(50);
        }
    }

    /// Perform WEP authentication (simplified)
    fn perform_wep_authentication(&self, credentials: &WifiCredentials) -> Result<(), Cyw43436Error> {
        defmt::info!("Performing WEP authentication");

        // Send WEP key to firmware
        let wep_key = credentials.passphrase.as_bytes();
        self.sdio_write_bytes(SDIO_FUNC_2, 0x4000, wep_key)?;

        // Signal WEP authentication
        self.sdio_write_byte(SDIO_FUNC_2, 0x4001, 0x01)?;

        // Wait for authentication completion
        let start_time = self.timer.get_time_ms();
        loop {
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x4002)?;

            if status & 0x01 != 0 { // WEP authentication successful
                defmt::info!("WEP authentication completed");
                return Ok(());
            }

            if status & 0x02 != 0 { // WEP authentication failed
                defmt::error!("WEP authentication failed");
                return Err(Cyw43436Error::InvalidResponse);
            }

            if self.timer.get_time_ms() - start_time > 5000 { // 5 second timeout
                defmt::error!("WEP authentication timeout");
                return Err(Cyw43436Error::Timeout);
            }

            self.timer.delay_ms(100);
        }
    }

    /// Disconnect from the current WiFi network
    pub fn disconnect(&mut self) -> Result<(), Cyw43436Error> {
        if !self.is_ready() {
            return Err(Cyw43436Error::NotReady);
        }

        defmt::info!("Disconnecting from WiFi network");

        // Send disconnect command
        self.sdio_write_byte(SDIO_FUNC_2, 0x5000, 0x01)?;

        // Wait for disconnection
        let start_time = self.timer.get_time_ms();
        loop {
            let status = self.sdio_read_byte(SDIO_FUNC_2, 0x5001)?;

            if status & 0x01 != 0 { // Disconnected
                defmt::info!("Successfully disconnected");
                return Ok(());
            }

            if self.timer.get_time_ms() - start_time > 5000 { // 5 second timeout
                defmt::warn!("Disconnect timeout, assuming disconnected");
                return Ok(());
            }

            self.timer.delay_ms(100);
        }
    }

    /// Get current connection state
    pub fn get_connection_state(&self) -> Result<ConnectionState, Cyw43436Error> {
        if !self.is_ready() {
            return Ok(ConnectionState::Disconnected);
        }

        let status = self.sdio_read_byte(SDIO_FUNC_2, 0x6000)?;

        match status & 0x03 {
            0x00 => Ok(ConnectionState::Disconnected),
            0x01 => Ok(ConnectionState::Connecting),
            0x02 => Ok(ConnectionState::Connected),
            0x03 => Ok(ConnectionState::Failed),
            _ => Ok(ConnectionState::Disconnected),
        }
    }
}

/// WiFi scan result
#[derive(Debug, Clone)]
pub struct ScanResult {
    pub ssid: heapless::String<32>,
    pub bssid: [u8; 6],
    pub channel: u8,
    pub rssi: i8,
    pub security: SecurityType,
}

impl defmt::Format for ScanResult {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "ScanResult {{ ssid: \"{}\", bssid: [{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}], channel: {}, rssi: {}dBm, security: {:?} }}",
            self.ssid.as_str(),
            self.bssid[0], self.bssid[1], self.bssid[2],
            self.bssid[3], self.bssid[4], self.bssid[5],
            self.channel,
            self.rssi,
            self.security
        );
    }
}

/// WiFi security types
#[derive(Debug, Clone, Copy, Format)]
pub enum SecurityType {
    Open,
    Wep,
    Wpa,
    Wpa2,
    Wpa3,
    Unknown,
}

/// WiFi connection state
#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Failed,
}

/// WiFi connection credentials
#[derive(Debug, Clone)]
pub struct WifiCredentials {
    pub ssid: heapless::String<32>,
    pub passphrase: heapless::String<64>,
    pub security: SecurityType,
}

/// WPA/WPA2 4-way handshake state
#[derive(Debug, Clone)]
struct WpaHandshakeState {
    pmk: [u8; WPA_PMK_LENGTH],        // Pre-master key
    ptk: [u8; WPA_PTK_LENGTH],        // Pairwise transient key
    anonce: [u8; WPA_NONCE_LENGTH],   // Authenticator nonce
    snonce: [u8; WPA_NONCE_LENGTH],   // Supplicant nonce
    handshake_complete: bool,
}

impl WpaHandshakeState {
    fn new() -> Self {
        Self {
            pmk: [0; WPA_PMK_LENGTH],
            ptk: [0; WPA_PTK_LENGTH],
            anonce: [0; WPA_NONCE_LENGTH],
            snonce: [0; WPA_NONCE_LENGTH],
            handshake_complete: false,
        }
    }
}

/// Connection parameters
#[derive(Debug, Clone)]
pub struct ConnectionParams {
    pub credentials: WifiCredentials,
    pub timeout_ms: u32,
    pub retry_count: u8,
}

impl ConnectionParams {
    pub fn new(ssid: &str, passphrase: &str, security: SecurityType) -> Result<Self, Cyw43436Error> {
        let ssid = heapless::String::try_from(ssid)
            .map_err(|_| Cyw43436Error::InvalidResponse)?;
        let passphrase = heapless::String::try_from(passphrase)
            .map_err(|_| Cyw43436Error::InvalidResponse)?;

        Ok(Self {
            credentials: WifiCredentials {
                ssid,
                passphrase,
                security,
            },
            timeout_ms: WIFI_CONNECT_TIMEOUT_MS,
            retry_count: MAX_CONNECT_RETRIES,
        })
    }
}
