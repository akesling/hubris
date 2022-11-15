// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Hardware revision-independent server code for UDP interactions
//!
//! This is in a separate module to avoid polluting `main.rs` with a bunch of
//! imports from the `transceiver_messages` crate; it simply adds more functions
//! to our existing `ServerImpl`.
use crate::ServerImpl;
use drv_sidecar_front_io::transceivers::{
    FpgaController, FpgaPortMasks, PortLocation,
};
use hubpack::SerializedSize;
use task_net_api::*;
use transceiver_messages::{
    message::*,
    mgmt::{MemoryRegion, UpperPage},
    Error, ModuleId,
};

/// Convert from the over-the-network type to our local types
fn unpack(m: ModuleId) -> Result<(FpgaController, FpgaPortMasks), Error> {
    let fpga = get_fpga(m)?;
    let mask = get_mask(m)?;
    Ok((fpga, mask))
}

/// Convert from the over-the-network type to our local FPGA type
fn get_fpga(m: ModuleId) -> Result<FpgaController, Error> {
    match m.fpga_id {
        0 => Ok(FpgaController::Left),
        1 => Ok(FpgaController::Right),
        i => Err(Error::InvalidFpga(i)),
    }
}

/// Convert from the over-the-network type to our local port mask type
fn get_mask(m: ModuleId) -> Result<FpgaPortMasks, Error> {
    let fpga_ports: u16 = m.ports.0;
    match m.fpga_id {
        0 => Ok(FpgaPortMasks {
            left: fpga_ports,
            right: 0,
        }),
        1 => Ok(FpgaPortMasks {
            left: 0,
            right: fpga_ports,
        }),
        i => Err(Error::InvalidFpga(i)),
    }
}

impl ServerImpl {
    /// Attempt to read and handle data from the `net` socket
    pub fn check_net(
        &mut self,
        rx_data_buf: &mut [u8],
        tx_data_buf: &mut [u8],
    ) {
        const SOCKET: SocketName = SocketName::transceivers;

        match self.net.recv_packet(
            SOCKET,
            LargePayloadBehavior::Discard,
            rx_data_buf,
        ) {
            Ok(mut meta) => {
                // Modify meta.size based on the output packet size
                meta.size = match hubpack::deserialize(rx_data_buf) {
                    Ok((msg, data)) => {
                        let (reply, data_len) = match self.handle_message(
                            msg,
                            data,
                            &mut tx_data_buf[Message::MAX_SIZE..],
                        ) {
                            Ok(r) => r,
                            Err(e) => (HostResponse::Error(e), 0),
                        };
                        let out = Message {
                            header: msg.header,
                            modules: msg.modules,
                            body: MessageBody::HostResponse(reply),
                        };
                        // Serialize into the front of the tx buffer
                        let msg_len =
                            hubpack::serialize(tx_data_buf, &out).unwrap();

                        // At this point, any supplementary data is written to
                        // tx_buf[Message::MAX_SIZE..].  Let's shift it
                        // backwards based on the size of the leading `Message`:
                        tx_data_buf.copy_within(
                            Message::MAX_SIZE..(Message::MAX_SIZE + data_len),
                            msg_len,
                        );
                        (msg_len + data_len) as u32
                    }
                    Err(_e) => hubpack::serialize(
                        tx_data_buf,
                        &Message {
                            header: Header {
                                version: 1,
                                message_id: u64::MAX,
                            },
                            modules: ModuleId::default(),
                            body: MessageBody::HostResponse(
                                HostResponse::Error(Error::ProtocolError),
                            ),
                        },
                    )
                    .unwrap() as u32,
                };

                self.net
                    .send_packet(
                        SOCKET,
                        meta,
                        &tx_data_buf[..meta.size as usize],
                    )
                    .unwrap();
            }
            Err(RecvError::QueueEmpty) => {
                // Our incoming queue is empty. Wait for more packets
                // in dispatch_n, back in the main loop.
            }
            Err(RecvError::NotYours | RecvError::Other) => panic!(),
        }
    }

    /// Handles a single message from the host with supplementary data in `data`
    ///
    /// Returns a response and a `usize` indicating how much was written to the
    /// `out` buffer.
    fn handle_message(
        &mut self,
        msg: Message,
        data: &[u8],
        out: &mut [u8],
    ) -> Result<(HostResponse, usize), Error> {
        if msg.header.version != 1 {
            return Err(Error::VersionMismatch);
        }

        match msg.body {
            MessageBody::SpRequest(..)
            | MessageBody::SpResponse(..)
            | MessageBody::HostResponse(..) => {
                return Err(Error::ProtocolError);
            }
            MessageBody::HostRequest(h) => {
                self.handle_host_request(h, msg.modules, data, out)
            }
        }
    }

    fn handle_host_request(
        &mut self,
        h: HostRequest,
        modules: ModuleId,
        data: &[u8],
        out: &mut [u8],
    ) -> Result<(HostResponse, usize), Error> {
        match h {
            HostRequest::Reset => {
                let mask = get_mask(modules)?;
                // TODO: use a more correct error code
                self.transceivers
                    .set_reset(mask)
                    .map_err(|_e| Error::ReadFailed)?;
                userlib::hl::sleep_for(1);
                self.transceivers
                    .clear_reset(mask)
                    .map_err(|_e| Error::ReadFailed)?;
                Ok((HostResponse::Ack, 0))
            }
            HostRequest::Status => {
                use drv_sidecar_front_io::Addr;
                use zerocopy::{BigEndian, U16};

                let fpga = get_fpga(modules)?;
                let fpga = self.transceivers.fpga(fpga);

                // This is a bit awkward: the FPGA will get _every_ module's
                // status (for the given FPGA), then we'll unpack to only the
                // ones that we care about
                let enable: U16<BigEndian> = fpga
                    .read(Addr::QSFP_CTRL_EN_H)
                    .map_err(|_e| Error::ReadFailed)?;
                let reset: U16<BigEndian> = fpga
                    .read(Addr::QSFP_CTRL_RESET_H)
                    .map_err(|_e| Error::ReadFailed)?;
                let lpmode: U16<BigEndian> = fpga
                    .read(Addr::QSFP_CTRL_LPMODE_H)
                    .map_err(|_e| Error::ReadFailed)?;
                let present: U16<BigEndian> = fpga
                    .read(Addr::QSFP_STATUS_PRESENT_H)
                    .map_err(|_e| Error::ReadFailed)?;
                let irq: U16<BigEndian> = fpga
                    .read(Addr::QSFP_STATUS_IRQ_H)
                    .map_err(|_e| Error::ReadFailed)?;

                // Write one bitfield per active port in the ModuleId
                let mut count = 0;
                for mask in modules.ports.to_indices().map(|i| 1 << i) {
                    let mut status = Status::empty();
                    if (enable.get() & mask) != 0 {
                        status |= Status::ENABLED;
                    }
                    if (reset.get() & mask) != 0 {
                        status |= Status::RESET;
                    }
                    if (lpmode.get() & mask) != 0 {
                        status |= Status::LOW_POWER_MODE;
                    }
                    if (present.get() & mask) != 0 {
                        status |= Status::PRESENT;
                    }
                    if (irq.get() & mask) != 0 {
                        status |= Status::INTERRUPT;
                    }
                    // Convert from Status -> u8 and write to the output buffer
                    out[count] = status.bits();
                    count += 1;
                }
                Ok((HostResponse::Status, count))
            }
            HostRequest::Read(mem) => {
                let out_size = mem.len() as u32 * modules.ports.0.count_ones();
                if out_size as usize > transceiver_messages::MAX_MESSAGE_SIZE {
                    return Err(Error::RequestTooLarge);
                }
                self.read(mem, modules, out)?;
                Ok((HostResponse::Read(mem), out_size as usize))
            }
            HostRequest::Write(mem) => {
                let data_size = mem.len() as u32 * modules.ports.0.count_ones();
                // TODO: check equality here and return a different error?
                if data_size as usize > data.len() {
                    return Err(Error::RequestTooLarge);
                }
                self.write(mem, modules, data)?;
                Ok((HostResponse::Write(mem), 0))
            }
            HostRequest::SetPowerMode(mode) => {
                let mask = get_mask(modules)?;
                // TODO: do we need delays in between any of these operations?
                match mode {
                    PowerMode::Off => {
                        // Power disabled, LpMode enabled (the latter doesn't
                        // make a difference, but keeps us in a known state)
                        self.transceivers
                            .clear_power_enable(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                        self.transceivers
                            .set_reset(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                        self.transceivers
                            .set_lpmode(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                    }
                    PowerMode::Low => {
                        // Power enabled, LpMode enabled
                        self.transceivers
                            .set_lpmode(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                        self.transceivers
                            .set_power_enable(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                        self.transceivers
                            .clear_reset(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                    }
                    PowerMode::High => {
                        // Power enabled, LpMode disabled
                        self.transceivers
                            .clear_lpmode(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                        self.transceivers
                            .set_power_enable(mask)
                            .map_err(|_e| Error::ReadFailed)?;
                    }
                }
                Ok((HostResponse::Ack, 0))
            }
            HostRequest::ManagementInterface(i) => {
                todo!()
            }
        }
    }

    fn select_page(
        &mut self,
        upper_page: UpperPage,
        modules: ModuleId,
    ) -> Result<(), Error> {
        let mask = get_mask(modules)?;

        match upper_page {
            UpperPage::Cmis(page) => {
                self.transceivers
                    .set_i2c_write_buffer(&[page.page()])
                    .map_err(|_e| Error::WriteFailed)?;
                self.transceivers
                    .setup_i2c_write(0x7F, 1, mask)
                    .map_err(|_e| Error::WriteFailed)?;
                self.wait_and_check_i2c(modules)?;

                if let Some(bank) = page.bank() {
                    self.transceivers
                        .set_i2c_write_buffer(&[bank])
                        .map_err(|_e| Error::ReadFailed)?;
                    self.transceivers
                        .setup_i2c_write(0x7E, 1, mask)
                        .map_err(|_e| Error::ReadFailed)?;
                    self.wait_and_check_i2c(modules)?;
                }
            }
            UpperPage::Sff8636(page) => {
                self.transceivers
                    .set_i2c_write_buffer(&[page.page()])
                    .map_err(|_e| Error::ReadFailed)?;
                self.transceivers
                    .setup_i2c_write(0x7F, 1, mask)
                    .map_err(|_e| Error::ReadFailed)?;
                self.wait_and_check_i2c(modules)?;
            }
        }
        Ok(())
    }

    fn wait_and_check_i2c(&mut self, modules: ModuleId) -> Result<(), Error> {
        // TODO: use a better error type here
        let (fpga, mask) = unpack(modules)?;
        self.transceivers
            .wait_for_i2c(fpga)
            .map_err(|_e| Error::WriteFailed)?;
        if let Some(p) = self
            .transceivers
            .get_i2c_error(fpga, mask)
            .map_err(|_e| Error::WriteFailed)?
        {
            // FPGA reported an I2C error
            return Err(Error::WriteFailed);
        }
        Ok(())
    }

    fn read(
        &mut self,
        mem: MemoryRegion,
        modules: ModuleId,
        out: &mut [u8],
    ) -> Result<(), Error> {
        // Reading beyond a 128-byte page boundary will wrap around; return an
        // error if that's going to happen.
        let page_end = (mem.offset() as u32 / 128 + 1) * 128;
        if mem.offset() as u32 + mem.len() as u32 > page_end {
            // TODO use a more descriptive error name here?
            return Err(Error::InvalidMemoryAccess {
                offset: mem.offset(),
                len: mem.len(),
            });
        }
        // CMIS spec only guarantee an 8-byte read (?!)
        // (see CMIS 5.0, 5.2.2.1)
        if matches!(mem.upper_page(), UpperPage::Cmis(..)) && mem.len() > 8 {
            // TODO: more specialized error
            return Err(Error::RequestTooLarge);
        }

        self.select_page(*mem.upper_page(), modules)?;
        self.transceivers
            .setup_i2c_read(mem.offset(), mem.len(), get_mask(modules)?)
            .map_err(|_e| Error::ReadFailed)?;
        self.wait_and_check_i2c(modules)?;

        let controller = get_fpga(modules)?;
        for (port, out) in modules
            .ports
            .to_indices()
            .zip(out.chunks_mut(mem.len() as usize))
        {
            self.transceivers
                .get_i2c_read_buffer(PortLocation { controller, port }, out)
                .map_err(|_e| Error::ReadFailed)?;
        }
        Ok(())
    }

    fn write(
        &mut self,
        mem: MemoryRegion,
        modules: ModuleId,
        data: &[u8],
    ) -> Result<(), Error> {
        // Writing beyond a 128-byte page boundary will wrap around; return an
        // error if that's going to happen.
        let page_end = (mem.offset() as u32 / 128 + 1) * 128;
        if mem.offset() as u32 + mem.len() as u32 > page_end {
            // TODO use a more descriptive error name here?
            return Err(Error::InvalidMemoryAccess {
                offset: mem.offset(),
                len: mem.len(),
            });
        }
        match *mem.upper_page() {
            // CMIS 5.0, §5.2.2.2: only guarantee an 8-byte write
            UpperPage::Cmis(..) => {
                if mem.len() > 8 {
                    // TODO: more specialized error
                    return Err(Error::RequestTooLarge);
                }
            }
            // SFF-8636, §5.3.3: max of 4 bytes in one write transaction
            UpperPage::Sff8636(..) => {
                if mem.len() > 4 {
                    // TODO: more specialized error
                    return Err(Error::RequestTooLarge);
                }
            }
        }

        self.select_page(*mem.upper_page(), modules)
            .map_err(|_e| Error::WriteFailed)?;

        // Copy data into the FPGA write buffer
        self.transceivers
            .set_i2c_write_buffer(&data[..mem.len() as usize])
            .map_err(|_e| Error::WriteFailed)?;

        // Trigger a multicast write to all transceivers in the mask
        self.transceivers
            .setup_i2c_write(mem.offset(), mem.len(), get_mask(modules)?)
            .map_err(|_e| Error::WriteFailed)?;
        self.wait_and_check_i2c(modules)?;

        Ok(())
    }
}
