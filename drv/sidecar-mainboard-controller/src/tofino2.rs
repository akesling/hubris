// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::{Addr, MainboardController, Reg};
use bitfield::bitfield;
use drv_fpga_api::{FpgaError, FpgaUserDesign, WriteOp};
use userlib::FromPrimitive;
use zerocopy::{AsBytes, FromBytes};

#[derive(Copy, Clone, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum TofinoSeqState {
    Initial = 0,
    A2 = 1,
    A0 = 2,
    InPowerUp = 3,
    InPowerDown = 4,
}

#[derive(Copy, Clone, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum TofinoSeqError {
    None = 0,
    PowerGoodTimeout = 1,
    PowerFault = 2,
    PowerVrHot = 3,
    PowerInvalidState = 4,
    UserAbort = 5,
    VidAckTimeout = 6,
    ThermalAlert = 7,
}

/// VID to voltage mapping. The VID values are specified in TF2-DS2, with the
/// actual voltage values derived experimentally after load testing the PDN.
#[derive(Copy, Clone, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum Tofino2Vid {
    V0P922 = 0b1111,
    V0P893 = 0b1110,
    V0P867 = 0b1101,
    V0P847 = 0b1100,
    V0P831 = 0b1011,
    V0P815 = 0b1010,
    V0P790 = 0b1001,
    V0P759 = 0b1000,
}

pub struct Sequencer {
    fpga: FpgaUserDesign,
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Status {
    state: TofinoSeqState,
    error: TofinoSeqError,
    vid: Option<Tofino2Vid>,
    power: u32,
    pcie_status: u8,
}

#[derive(Copy, Clone, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum TofinoPcieReset {
    HostControl,
    Asserted,
    Deasserted,
}

impl Sequencer {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self {
            fpga: FpgaUserDesign::new(
                task_id,
                MainboardController::DEVICE_INDEX,
            ),
        }
    }

    fn read_masked(&self, addr: Addr, mask: u8) -> Result<u8, FpgaError> {
        let v: u8 = self.fpga.read(addr)?;
        Ok(v & mask)
    }

    fn write_ctrl(&self, op: WriteOp, value: u8) -> Result<(), FpgaError> {
        self.fpga.write(op, Addr::TOFINO_SEQ_CTRL, value)
    }

    pub fn clear_error(&self) -> Result<(), FpgaError> {
        self.write_ctrl(WriteOp::BitSet, Reg::TOFINO_SEQ_CTRL::CLEAR_ERROR)
    }

    pub fn enabled(&self) -> Result<bool, FpgaError> {
        self.read_masked(Addr::TOFINO_SEQ_CTRL, Reg::TOFINO_SEQ_CTRL::EN)
            .map(|v| v != 0)
    }

    pub fn set_enable(&self, enabled: bool) -> Result<(), FpgaError> {
        let op = if enabled {
            WriteOp::BitSet
        } else {
            WriteOp::BitClear
        };
        self.write_ctrl(op, Reg::TOFINO_SEQ_CTRL::EN)
    }

    pub fn ack_vid(&self) -> Result<(), FpgaError> {
        self.write_ctrl(WriteOp::BitSet, Reg::TOFINO_SEQ_CTRL::ACK_VID)
    }

    pub fn state(&self) -> Result<TofinoSeqState, FpgaError> {
        let v = self.read_masked(
            Addr::TOFINO_SEQ_STATE,
            Reg::TOFINO_SEQ_STATE::STATE,
        )?;
        TofinoSeqState::from_u8(v).ok_or(FpgaError::InvalidValue)
    }

    pub fn error(&self) -> Result<TofinoSeqError, FpgaError> {
        let v = self.read_masked(
            Addr::TOFINO_SEQ_ERROR,
            Reg::TOFINO_SEQ_ERROR::ERROR,
        )?;
        TofinoSeqError::from_u8(v).ok_or(FpgaError::InvalidValue)
    }

    pub fn power_status(&self) -> Result<u32, FpgaError> {
        self.fpga.read(Addr::TOFINO_POWER_ENABLE)
    }

    /// The VID is only valid once Tofino is powered up and a delay after PoR
    /// has lapsed. If the VID is read while in this state a `Some(..)` will be
    /// returned. Attempting to read the VID outside this window will result in
    /// `None`. An `FpgaError` is returned if communication with the mainboard
    /// controller failed or an invalid value was read from the register.
    pub fn vid(&self) -> Result<Option<Tofino2Vid>, FpgaError> {
        let v: u8 = self.fpga.read(Addr::TOFINO_POWER_VID)?;

        if (v & Reg::TOFINO_POWER_VID::VID_VALID) != 0 {
            match Tofino2Vid::from_u8(v & Reg::TOFINO_POWER_VID::VID) {
                None => Err(FpgaError::InvalidValue),
                some_vid => Ok(some_vid),
            }
        } else {
            Ok(None)
        }
    }

    pub fn pcie_hotplug_ctrl(&self) -> Result<u8, FpgaError> {
        self.fpga.read(Addr::PCIE_HOTPLUG_CTRL)
    }

    pub fn write_pcie_hotplug_ctrl(
        &self,
        op: WriteOp,
        value: u8,
    ) -> Result<(), FpgaError> {
        self.fpga.write(op, Addr::PCIE_HOTPLUG_CTRL, value)
    }

    pub fn set_pcie_present(&self, present: bool) -> Result<(), FpgaError> {
        self.write_pcie_hotplug_ctrl(
            present.into(),
            Reg::PCIE_HOTPLUG_CTRL::PRESENT,
        )
    }

    pub fn set_pcie_power_fault(&self, fault: bool) -> Result<(), FpgaError> {
        self.write_pcie_hotplug_ctrl(
            fault.into(),
            Reg::PCIE_HOTPLUG_CTRL::POWER_FAULT,
        )
    }

    pub fn set_pcie_alert(&self, alert: bool) -> Result<(), FpgaError> {
        self.write_pcie_hotplug_ctrl(
            alert.into(),
            Reg::PCIE_HOTPLUG_CTRL::ALERT,
        )
    }

    pub fn pcie_reset(&self) -> Result<TofinoPcieReset, FpgaError> {
        let ctrl = self.pcie_hotplug_ctrl()?;
        let reset = (ctrl & Reg::PCIE_HOTPLUG_CTRL::RESET) != 0;
        let override_host_reset =
            (ctrl & Reg::PCIE_HOTPLUG_CTRL::OVERRIDE_HOST_RESET) != 0;

        match (override_host_reset, reset) {
            (false, _) => Ok(TofinoPcieReset::HostControl),
            (true, false) => Ok(TofinoPcieReset::Deasserted),
            (true, true) => Ok(TofinoPcieReset::Asserted),
        }
    }

    pub fn set_pcie_reset(
        &self,
        reset: TofinoPcieReset,
    ) -> Result<(), FpgaError> {
        let ctrl = self.pcie_hotplug_ctrl()?;
        let ctrl_next = match reset {
            TofinoPcieReset::HostControl => {
                // Clear RESET, OVERRIDE_HOST_RESET.
                ctrl & !(Reg::PCIE_HOTPLUG_CTRL::RESET
                    | Reg::PCIE_HOTPLUG_CTRL::OVERRIDE_HOST_RESET)
            }
            TofinoPcieReset::Asserted => {
                // Set RESET, OVERRIDE_HOST_RESET.
                ctrl | Reg::PCIE_HOTPLUG_CTRL::RESET
                    | Reg::PCIE_HOTPLUG_CTRL::OVERRIDE_HOST_RESET
            }
            TofinoPcieReset::Deasserted => {
                // Set HOST_OVERRIDE_RESET, clear RESET.
                (ctrl & !Reg::PCIE_HOTPLUG_CTRL::RESET)
                    | Reg::PCIE_HOTPLUG_CTRL::OVERRIDE_HOST_RESET
            }
        };

        self.write_pcie_hotplug_ctrl(WriteOp::Write, ctrl_next)
    }

    pub fn pcie_hotplug_status(&self) -> Result<u8, FpgaError> {
        self.fpga.read(Addr::PCIE_HOTPLUG_STATUS)
    }

    pub fn status(&self) -> Result<Status, FpgaError> {
        Ok(Status {
            state: self.state()?,
            error: self.error()?,
            vid: self.vid()?,
            power: self.power_status()?,
            pcie_status: self.pcie_hotplug_status()?,
        })
    }
}

bitfield! {
    #[derive(Copy, Clone, PartialEq, Eq, FromPrimitive, AsBytes, FromBytes)]
    #[repr(C)]
    pub struct DebugPortState(u8);
    pub send_buffer_empty, set_send_buffer_empty: 0;
    pub send_buffer_full, _: 1;
    pub receive_buffer_empty, set_receive_buffer_empty: 2;
    pub receive_buffer_full, _: 3;
    pub request_in_progress, set_request_in_progress: 4;
    pub address_nack_error, set_address_nack_error: 5;
    pub byte_nack_error, set_byte_nack_error: 6;
}

#[derive(Copy, Clone, PartialEq, Eq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum DebugRequestOpcode {
    LocalWrite = 0b0000_0000,
    LocalRead = 0b0010_0000,
    DirectWrite = 0b1000_0000,
    DirectRead = 0b1010_0000,
    IndirectWrite = 0b1100_0000,
    IndirectRead = 0b1110_0000,
}

impl From<DebugRequestOpcode> for u8 {
    fn from(opcode: DebugRequestOpcode) -> Self {
        opcode as u8
    }
}

#[derive(Copy, Clone, PartialEq, Eq, FromPrimitive, AsBytes)]
#[repr(u32)]
pub enum DirectBarSegment {
    Bar0 = 0,
    Msi = 1 << 28,
    Cfg = 2 << 28,
}

/// A few of the Tofino registers which are used in code below. These are found
/// in 631384-0001_TF2-Top-Level_Register_Map_05062021.html as provided by
/// Intel.
#[derive(Copy, Clone, PartialEq, Eq, FromPrimitive, AsBytes)]
#[repr(u32)]
pub enum TofinoRegisters {
    Scratchpad = 0x0,
    FreeRunningCounter = 0x10,
    SpiOutData0 = (0x80000 | 0x120),
    SpiOutData1 = (0x80000 | 0x124),
    SpiInData = (0x80000 | 0x12c),
    SpiCommand = (0x80000 | 0x128),
    SpiIdCode = (0x80000 | 0x130),
}

impl From<TofinoRegisters> for u32 {
    fn from(r: TofinoRegisters) -> Self {
        r as u32
    }
}

/// SPI EEPROM instructions, as per for example
/// https://octopart.com/datasheet/cat25512vi-gt3-onsemi-22302617.
#[derive(Copy, Clone, PartialEq, Eq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum SpiEepromInstruction {
    // WREN, enable write operations
    WriteEnable = 0x6,
    // WRDI, disable write operations
    WriteDisable = 0x4,
    // RDSR, read Status register
    ReadStatusRegister = 0x5,
    // WRSR, write Status register. See datasheet for which bits can actually be
    // written.
    WriteStatusRegister = 0x1,
    // READ, read a number of bytes from the EEPROM
    Read = 0x3,
    // WRITE, write a number of bytes to the EEPROM
    Write = 0x2,
}

impl From<SpiEepromInstruction> for u8 {
    fn from(i: SpiEepromInstruction) -> Self {
        i as u8
    }
}

pub struct DebugPort {
    fpga: FpgaUserDesign,
}

impl DebugPort {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self {
            fpga: FpgaUserDesign::new(
                task_id,
                MainboardController::DEVICE_INDEX,
            ),
        }
    }

    pub fn state(&self) -> Result<DebugPortState, FpgaError> {
        self.fpga.read(Addr::TOFINO_DEBUG_PORT_STATE)
    }

    pub fn set_state(&self, state: DebugPortState) -> Result<(), FpgaError> {
        self.fpga
            .write(WriteOp::Write, Addr::TOFINO_DEBUG_PORT_STATE, state)
    }

    pub fn read_direct(
        &self,
        segment: DirectBarSegment,
        offset: impl Into<u32> + Copy,
    ) -> Result<u32, FpgaError> {
        assert!(offset.into() < 1 << 28);

        let state = self.state()?;
        if !state.send_buffer_empty() || !state.receive_buffer_empty() {
            return Err(FpgaError::InvalidState);
        }

        // Add the segement base address to the given read offset.
        let address = segment as u32 | offset.into();

        // Write the opcode.
        self.fpga.write(
            WriteOp::Write,
            Addr::TOFINO_DEBUG_PORT_BUFFER,
            DebugRequestOpcode::DirectRead,
        )?;

        // Write the address. This is done in a loop because the SPI peripheral
        // in the FPGA auto-increments an address pointer. This will be
        // refactored when a non auto-incrementing `WriteOp` is implemented.
        for b in address.as_bytes().iter() {
            self.fpga.write(
                WriteOp::Write,
                Addr::TOFINO_DEBUG_PORT_BUFFER,
                *b,
            )?;
        }

        // Start the request.
        self.fpga.write(
            WriteOp::Write,
            Addr::TOFINO_DEBUG_PORT_STATE,
            Reg::TOFINO_DEBUG_PORT_STATE::REQUEST_IN_PROGRESS,
        )?;

        // Wait for the request to complete.
        while self.state()?.request_in_progress() {
            userlib::hl::sleep_for(1);
        }

        // Read the response. This is done in a loop because the SPI peripheral
        // in the FPGA auto-increments an address pointer. This will be
        // refactored when a non auto-incrementing `read(..)` is implemented.
        let mut v: u32 = 0;
        for b in v.as_bytes_mut().iter_mut() {
            *b = self.fpga.read(Addr::TOFINO_DEBUG_PORT_BUFFER)?;
        }

        Ok(v)
    }

    pub fn write_direct(
        &self,
        segment: DirectBarSegment,
        offset: impl Into<u32> + Copy,
        value: u32,
    ) -> Result<(), FpgaError> {
        assert!(offset.into() < 1 << 28);

        let state = self.state()?;
        if !state.send_buffer_empty() || !state.receive_buffer_empty() {
            return Err(FpgaError::InvalidState);
        }

        // Add the segement base address to the given read offset.
        let address = segment as u32 | offset.into();

        // Write the opcode to the queue.
        self.fpga.write(
            WriteOp::Write,
            Addr::TOFINO_DEBUG_PORT_BUFFER,
            DebugRequestOpcode::DirectWrite,
        )?;

        // Write the address to the queue. This is done in a loop because the
        // SPI peripheral in the FPGA auto-increments an address pointer. This
        // will be refactored when a non auto-incrementing `WriteOp` is
        // implemented.
        for b in address.as_bytes().iter() {
            self.fpga.write(
                WriteOp::Write,
                Addr::TOFINO_DEBUG_PORT_BUFFER,
                *b,
            )?;
        }

        // Write the value to the queue.
        for b in value.as_bytes().iter() {
            self.fpga.write(
                WriteOp::Write,
                Addr::TOFINO_DEBUG_PORT_BUFFER,
                *b,
            )?;
        }

        // Start the request.
        self.fpga.write(
            WriteOp::Write,
            Addr::TOFINO_DEBUG_PORT_STATE,
            Reg::TOFINO_DEBUG_PORT_STATE::REQUEST_IN_PROGRESS,
        )?;

        // Wait for the request to complete.
        while self.state()?.request_in_progress() {
            userlib::hl::sleep_for(1);
        }

        Ok(())
    }

    /// Generate the SPI command Tofino needs to complete a SPI request.
    fn spi_command(n_bytes_to_write: usize, n_bytes_to_read: usize) -> u32 {
        assert!(n_bytes_to_write <= 8);
        assert!(n_bytes_to_read <= 4);

        (0x80 | ((n_bytes_to_read & 0x7) << 4) | (n_bytes_to_write & 0xf))
            .try_into()
            .unwrap()
    }

    /// Wait for a SPI request to complete.
    fn await_spi_request_done(&self) -> Result<(), FpgaError> {
        while self
            .read_direct(DirectBarSegment::Bar0, TofinoRegisters::SpiCommand)?
            & 0x80
            != 0
        {
            userlib::hl::sleep_for(1);
        }

        Ok(())
    }

    /// Send an instruction to the Tofino attached SPI EEPROM.
    pub fn send_spi_eeprom_instruction(
        &self,
        i: SpiEepromInstruction,
    ) -> Result<(), FpgaError> {
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiOutData0,
            i as u32,
        )?;
        // Initiate the SPI transaction.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiCommand,
            Self::spi_command(1, 1),
        )?;

        self.await_spi_request_done()
    }

    /// Read the register containing the IDCODE latched by Tofino when it
    /// successfully reads the PCIe SerDes parameters from the SPI EEPROM.
    pub fn spi_eeprom_idcode(&self) -> Result<u32, FpgaError> {
        self.read_direct(DirectBarSegment::Bar0, TofinoRegisters::SpiIdCode)
    }

    /// Read the SPI EEPROM Status register.
    pub fn spi_eeprom_status(&self) -> Result<u8, FpgaError> {
        self.send_spi_eeprom_instruction(
            SpiEepromInstruction::ReadStatusRegister,
        )?;

        // Read the EEPROM response.
        Ok(self
            .read_direct(DirectBarSegment::Bar0, TofinoRegisters::SpiInData)?
            as u8)
    }

    /// Write the SPI EEPROM Status register.
    pub fn set_spi_eeprom_status(&self, value: u8) -> Result<(), FpgaError> {
        // Request the WRSR instruction with the given value.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiOutData0,
            u32::from_le_bytes([
                SpiEepromInstruction::WriteStatusRegister as u8,
                value,
                0,
                0,
            ]),
        )?;
        // Initiate the SPI transaction.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiCommand,
            Self::spi_command(1, 0),
        )?;

        self.await_spi_request_done()
    }

    /// Read four bytes from the SPI EEPROM at the given offset.
    pub fn read_spi_eeprom(&self, offset: usize) -> Result<[u8; 4], FpgaError> {
        // Request a read of the given address.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiOutData0,
            u32::from_le_bytes([
                SpiEepromInstruction::Read as u8,
                (offset >> 8) as u8,
                offset as u8,
                0,
            ]),
        )?;

        // Initiate the SPI transaction.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiCommand,
            Self::spi_command(3, 4),
        )?;

        self.await_spi_request_done()?;

        // Read the EEPROM response.
        Ok(self
            .read_direct(DirectBarSegment::Bar0, TofinoRegisters::SpiInData)?
            .to_be_bytes())
    }

    /// Write four bytes into the SPI EEPROM at the given offset.
    pub fn write_spi_eeprom(
        &self,
        offset: usize,
        data: [u8; 4],
    ) -> Result<(), FpgaError> {
        self.send_spi_eeprom_instruction(SpiEepromInstruction::WriteEnable)?;

        // Request a Write of the given address.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiOutData0,
            u32::from_le_bytes([
                SpiEepromInstruction::Write as u8,
                (offset >> 8) as u8,
                offset as u8,
                data[0],
            ]),
        )?;

        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiOutData1,
            u32::from_le_bytes([data[1], data[2], data[3], 0]),
        )?;

        // Initiate the SPI transaction.
        self.write_direct(
            DirectBarSegment::Bar0,
            TofinoRegisters::SpiCommand,
            Self::spi_command(7, 0),
        )?;

        self.await_spi_request_done()
    }

    /// Read the requested number of bytes from the SPI EEPROM at the given
    /// offset into the given byte buffer. Note that the given offset needs to
    /// be four-byte aligned.
    pub fn read_spi_eeprom_bytes(
        &self,
        offset: usize,
        data: &mut [u8],
    ) -> Result<(), FpgaError> {
        // Only 4 byte aligned reads/writes are allowed.
        if offset % 4 != 0 {
            return Err(FpgaError::InvalidValue);
        }

        for (i, chunk) in data.chunks_mut(4).enumerate() {
            self.read_spi_eeprom(offset + (i * 4))
                .map(|bytes| chunk.copy_from_slice(&bytes[0..chunk.len()]))?;
        }

        Ok(())
    }

    /// Write the contents of the given byte buffer into the SPI EEPROM at the
    /// given offset. Note that the given offset needs to be four-byte aligned.
    pub fn write_spi_eeprom_bytes(
        &self,
        offset: usize,
        data: &[u8],
    ) -> Result<(), FpgaError> {
        // Only 4 byte aligned reads/writes are allowed.
        if offset % 4 != 0 {
            return Err(FpgaError::InvalidValue);
        }

        let mut bytes = [0u8; 4];
        for (i, chunk) in data.chunks(4).enumerate() {
            bytes[0..chunk.len()].copy_from_slice(chunk);
            self.write_spi_eeprom(offset + (i * 4), bytes)?;
        }

        Ok(())
    }
}
