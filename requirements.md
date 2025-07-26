# CYW43436 WiFi Driver Requirements for Raspberry Pi Zero 2 W (Bare Metal)

## Project Overview
Develop a Rust-based WiFi driver for the CYW43436 chip used in the Raspberry Pi Zero 2 W, specifically designed for bare metal embedded systems without an operating system. This driver will provide low-level hardware abstraction and WiFi functionality in a `no_std` environment.

## Hardware Requirements

### Target Hardware
- **Primary Target**: Raspberry Pi Zero 2 W (Bare Metal)
- **SoC**: Broadcom BCM2710A1 (ARM Cortex-A53 quad-core)
- **WiFi Chip**: Broadcom CYW43436 (802.11 b/g/n)
- **Interface**: SDIO (Secure Digital Input Output)
- **Memory**: 512MB LPDDR2 SDRAM
- **Additional**: GPIO pins for power management and control

### Development Hardware
- Raspberry Pi Zero 2 W development board
- MicroSD card (16GB+ recommended)
- USB-to-Serial adapter for debugging
- Logic analyzer (optional, for SDIO debugging)
- Oscilloscope (optional, for signal analysis)

## Software Requirements

### Development Environment
- **Language**: Rust (edition 2021+)
- **Target**: `armv7-unknown-linux-gnueabihf` (development) + bare metal target
- **Bare Metal Target**: Custom target for BCM2710/RPi Zero 2 W
- **Cross-compilation**: ARM cross-compiler toolchain
- **Build System**: Cargo with custom build scripts for bare metal
- **Boot Loader**: Custom bootloader or chainload from GPU firmware

### Core Dependencies (Bare Metal Focus)
```toml
[dependencies]
# Core embedded dependencies
embedded-hal = { version = "1.0", default-features = false }
embedded-hal-nb = { version = "1.0", default-features = false }
nb = { version = "1.1", default-features = false }

# Memory management for bare metal
linked_list_allocator = "0.10"
heapless = { version = "0.8", default-features = false }

# SDIO communication (bare metal compatible)
embedded-sdmmc = { version = "0.7", default-features = false, optional = true }

# Networking stack (no_std compatible)
smoltcp = { version = "0.11", default-features = false, features = ["proto-ipv4", "socket-tcp", "socket-udp"] }

# Async runtime for embedded
embassy-executor = { version = "0.5", default-features = false, features = ["arch-cortex-m", "executor-thread"] }
embassy-time = { version = "0.3", default-features = false }
embassy-sync = { version = "0.6", default-features = false }

# Logging and debugging (no_std)
defmt = { version = "0.3", default-features = false }
defmt-rtt = { version = "0.4", default-features = false }
panic-probe = { version = "0.3", features = ["print-defmt"] }

# Cryptography for WPA/WPA2 (no_std)
aes = { version = "0.8", default-features = false }
sha2 = { version = "0.10", default-features = false }
hmac = { version = "0.12", default-features = false }

# Binary manipulation (no_std)
byteorder = { version = "1.4", default-features = false }
bitflags = { version = "2.4", default-features = false }

# BCM2710 specific hardware abstraction
rpi-hal = { git = "https://github.com/rust-embedded/rpi-hal", optional = true }

[build-dependencies]
cc = "1.0"

[profile.dev]
debug = true
opt-level = 1

[profile.release]
debug = true
opt-level = "s"    # Optimize for size
lto = true
codegen-units = 1
```

## Bare Metal Specific Requirements

### 1. Memory Management
- **No Heap by Default**: Use stack-allocated data structures where possible
- **Custom Allocator**: Implement linked-list allocator for dynamic allocation when needed
- **Memory Layout**: Define custom memory layout for BCM2710
- **Stack Management**: Proper stack setup for each CPU core
- **DMA Coherency**: Handle cache coherency for DMA operations

### 2. Hardware Abstraction Layer (HAL)
- **MMIO Access**: Direct memory-mapped I/O for hardware registers
- **GPIO Control**: Direct GPIO register manipulation for power/reset
- **SDIO Controller**: Bare metal SDIO controller implementation
- **Interrupt Controller**: GIC (Generic Interrupt Controller) integration
- **Timer Management**: System timer for timeouts and delays
- **Cache Management**: L1/L2 cache control for ARM Cortex-A53

### 3. Boot and Initialization Sequence
- **Boot Process**: 
  1. GPU firmware loads kernel8.img
  2. ARM cores start execution
  3. Initialize memory management unit (MMU)
  4. Set up exception vectors
  5. Initialize hardware peripherals
  6. Start WiFi driver initialization
- **Multi-core Support**: Initialize all four ARM cores if needed
- **Clock Management**: Configure system clocks and PLLs

### 4. Custom Memory Layout
```rust
// memory.x - Linker script for RPi Zero 2 W
MEMORY {
    /* BCM2710 memory layout */
    RAM : ORIGIN = 0x00080000, LENGTH = 512M - 0x80000  /* Avoid GPU memory */
    GPU : ORIGIN = 0x40000000, LENGTH = 64M              /* GPU peripheral base */
}

SECTIONS {
    .text : {
        KEEP(*(.boot))
        *(.text*)
    } > RAM
    
    .data : {
        *(.data*)
    } > RAM
    
    .bss : {
        *(.bss*)
    } > RAM
}
```

## Technical Requirements (Bare Metal Specific)

### 1. Hardware Initialization
- **SDIO Controller Setup**: Configure EMMC2 controller for SDIO mode
- **Pin Multiplexing**: Configure GPIO pins for SDIO functionality
- **Power Management**: Control CYW43436 power via GPIO
- **Clock Configuration**: Set appropriate SDIO clock frequencies
- **Voltage Control**: Manage I/O voltage levels

### 2. Interrupt Handling
- **GIC Setup**: Configure Generic Interrupt Controller
- **SDIO Interrupts**: Handle SDIO card interrupts
- **GPIO Interrupts**: Handle wake-up and status interrupts
- **Timer Interrupts**: System tick for timeouts
- **Exception Vectors**: ARM exception handling

### 3. SDIO Protocol (Bare Metal)
- **Register-Level Access**: Direct EMMC2 register manipulation
- **Command Processing**: Hardware command queue management
- **Data Transfer**: DMA-based or programmed I/O data transfer
- **Error Recovery**: Hardware-level error detection and recovery
- **Timing Control**: Precise timing for SDIO operations

### 4. Memory and Cache Management
- **Cache Coherency**: Ensure DMA coherency with CPU caches
- **Memory Barriers**: ARM memory barrier instructions
- **Virtual Memory**: Optional MMU setup for memory protection
- **Buffer Alignment**: Align buffers for DMA requirements

## Implementation Phases (Bare Metal Focus)

### Phase 1: Bare Metal Foundation (Weeks 1-3)
- [ ] Boot loader and initialization code
- [ ] Memory layout and linker script
- [ ] Basic UART output for debugging
- [ ] GPIO and peripheral access
- [ ] Interrupt controller setup
- [ ] System timer implementation

### Phase 2: SDIO Hardware Layer (Weeks 4-6)
- [ ] EMMC2 controller initialization
- [ ] SDIO register-level interface
- [ ] Basic SDIO command/response
- [ ] DMA setup for data transfer
- [ ] Hardware interrupt handling

### Phase 3: CYW43436 Integration (Weeks 7-9)
- [ ] Chip detection and identification
- [ ] Firmware loading from embedded blob
- [ ] Basic chip initialization
- [ ] Register read/write operations
- [ ] Status monitoring

### Phase 4: WiFi Protocol Implementation (Weeks 10-13)
- [ ] 802.11 frame processing
- [ ] Scanning and association
- [ ] Authentication mechanisms
- [ ] Basic data path implementation
- [ ] Event handling

### Phase 5: Security and Networking (Weeks 14-16)
- [ ] WPA/WPA2 implementation
- [ ] TCP/IP stack integration
- [ ] Network interface abstraction
- [ ] Application-level API

### Phase 6: Optimization and Testing (Weeks 17-18)
- [ ] Performance optimization
- [ ] Power management
- [ ] Comprehensive testing on hardware
- [ ] Documentation and examples

## Development Steps (Bare Metal)

### Step 1: Bare Metal Environment Setup
1. Create custom target specification for RPi Zero 2 W
2. Set up cross-compilation toolchain
3. Configure debugging with OpenOCD/JTAG
4. Create bootloader and basic initialization

### Step 2: Hardware Access Layer
1. Implement MMIO abstraction
2. Create GPIO control functions
3. Set up interrupt handling
4. Implement system timer

### Step 3: SDIO Controller Implementation
1. Study BCM2710 EMMC2 controller documentation
2. Implement register-level SDIO interface
3. Create command/response handling
4. Implement DMA data transfer

### Step 4: CYW43436 Chip Driver
1. Embed firmware blob in binary
2. Implement chip initialization sequence
3. Create register access functions
4. Handle chip interrupts and events

### Step 5: WiFi Protocol Stack
1. Implement 802.11 management frames
2. Create scanning and association logic
3. Implement security protocols
4. Create data transmission path

### Step 6: Network Integration
1. Integrate with smoltcp TCP/IP stack
2. Create network interface abstraction
3. Implement application-level API
4. Add comprehensive error handling

## Bare Metal Specific Challenges

### Technical Challenges
- **No Standard Library**: All functionality must be implemented from scratch
- **Hardware Documentation**: Limited BCM2710 documentation availability
- **Debugging Complexity**: Limited debugging options without OS
- **Memory Constraints**: Efficient memory usage without heap allocator
- **Real-time Requirements**: Precise timing without OS scheduler

### Solutions and Mitigations
- **Extensive Testing**: Use QEMU emulation for initial development
- **Incremental Development**: Build and test each layer independently  
- **Hardware Debugging**: Use JTAG/SWD for low-level debugging
- **Code Reuse**: Study existing bare metal implementations
- **Community Support**: Leverage Rust embedded community resources

## Success Criteria (Bare Metal)

### Minimum Viable Product (MVP)
- [ ] Boot successfully on RPi Zero 2 W hardware
- [ ] Initialize CYW43436 chip without errors
- [ ] Scan for WiFi networks
- [ ] Connect to open networks
- [ ] Send/receive basic network packets

### Full Feature Set
- [ ] Complete WPA/WPA2 security support
- [ ] Stable long-term operation
- [ ] Power management features
- [ ] Performance comparable to Linux drivers
- [ ] Comprehensive API for applications

## Additional Resources (Bare Metal)

### Hardware Documentation
- BCM2710 ARM Peripherals datasheet
- ARM Cortex-A53 Technical Reference Manual
- SDIO/SD specifications
- CYW43436 reference materials

### Development Tools
- OpenOCD for JTAG debugging
- QEMU for emulation testing
- Logic analyzer for signal debugging
- ARM cross-compiler toolchain

### Reference Implementations
- Circle C++ bare metal framework for Raspberry Pi
- Rust embedded HAL implementations
- Linux brcmfmac driver source code
- Other bare metal WiFi implementations
