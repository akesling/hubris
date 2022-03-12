// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// use idol_runtime::{ClientError, Leased, LenLimit, RequestError, R, W};
use idol_runtime::{ClientError, Leased, RequestError, R, W};
use task_spdm_api::SpdmError;
use userlib::*;

use ringbuf::*;
use spdm::{
    config::NUM_SLOTS,
    crypto::{FakeSigner, FilledSlot},
    responder::AllStates,
};

#[derive(Copy, Clone, PartialEq, Debug)]
enum State {
    Error,
    Version,
    Capabilities,
    Algorithms,
    IdAuth,
    Challenge,
}

impl From<&AllStates> for State {
    fn from(state: &AllStates) -> Self {
        match state {
            AllStates::Error => State::Error,
            AllStates::Version(_) => State::Version,
            AllStates::Capabilities(_) => State::Capabilities,
            AllStates::Algorithms(_) => State::Algorithms,
            AllStates::IdAuth(_) => State::IdAuth,
            AllStates::Challenge(_) => State::Challenge,
        }
    }
}

/// Record the types and sizes of the messages sent and received by this server
#[derive(Copy, Clone, PartialEq, Debug)]
enum LogMsg {
    // Static initializer
    Init,
    State(State),
    RspLen(usize),
}
ringbuf!(LogMsg, 16, LogMsg::Init);

const MAX_SPDM_MSG_SIZE: usize = 256;

#[export_name = "main"]
fn main() -> ! {
    let mut buffer = [0; idl::INCOMING_SIZE];
    let mut server = ServerImpl::new();

    loop {
        idol_runtime::dispatch(&mut buffer, &mut server);
    }
}

struct ServerImpl<'a> {
    responder: spdm::Responder<'a, FakeSigner>,
    valid: usize,
    message: [u8; MAX_SPDM_MSG_SIZE],
}

impl<'a> ServerImpl<'a> {
    fn new() -> ServerImpl<'a> {
        const EMPTY_SLOT: Option<FilledSlot<'_, FakeSigner>> = None;
        let slots = [EMPTY_SLOT; NUM_SLOTS];
        let responder = spdm::Responder::new(slots);
        ringbuf_entry!(LogMsg::State(responder.state().into()));
        ServerImpl { responder, valid: 0, message: [0; MAX_SPDM_MSG_SIZE] }
    }
}

impl idl::InOrderSpdmImpl for ServerImpl<'_> {
    /// A client sends a message for SPDM processing.
    fn send(
        &mut self,
        _: &RecvMessage,
        length: usize,
        // source: LenLimit<Leased<R, [u8]>, 256>,
        source: Leased<R, [u8]>,
    ) -> Result<(), RequestError<SpdmError>> {
        if self.valid > 0 {
            return Err(SpdmError::MessageAlreadyExists.into());
        }
        if source.len() == 0 || source.len() < length {
            return Err(SpdmError::ShortMessage.into());
        }
        if length > self.message.len() {
            return Err(SpdmError::SourceTooLarge.into());
        }

        // Read the entire message into our address space.
        source
            .read_range(0..length, &mut self.message[..length])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;
        self.valid = length; // Message is ready for processing.

        // TODO: Replace toy transform with SPDM work.
        for byte in self.message[..length].iter_mut() {
            *byte = *byte ^ 0xff;
        }
        Ok(())
    }

    /// A client requests a response, or a client could be in the role
    /// of a responder and needs to poll for a request.
    fn recv(
        &mut self,
        _: &RecvMessage,
        // sink: LenLimit<Leased<W, [u8]>, 256>,
        sink: Leased<W, [u8]>,
    ) -> Result<usize, RequestError<SpdmError>> {
        if self.valid == 0 {
            return Err(SpdmError::NoMessageAvailable.into());
        }
        if self.valid > sink.len() {
            // Insufficient space to work in.
            return Err(SpdmError::SinkTooSmall.into());
        }
        let len = self.valid;
        sink.write_range(0..len, &self.message[..len])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;
        self.message[..self.valid].iter_mut().for_each(|m| *m = 0); // hygene: zero message
        self.valid = 0; // Comsume the stored message

        Ok(len)
    }

    // An SPDM client sends and receives messages.
    fn exchange(
        &mut self,
        _: &RecvMessage,
        length: usize,
        source: Leased<R, [u8]>,
        sink: Leased<W, [u8]>,
    ) -> Result<usize, RequestError<SpdmError>> {
        // Checks on sent message length
        if self.valid > 0 {
            return Err(SpdmError::MessageAlreadyExists.into());
        }
        if source.len() == 0 || source.len() < length {
            return Err(SpdmError::ShortMessage.into());
        }
        if length > self.message.len() {
            return Err(SpdmError::SourceTooLarge.into());
        }
        // Checks on receive message length
        if length > sink.len() {
            return Err(SpdmError::SinkTooSmall.into());
        }

        let mut req = [0u8; MAX_SPDM_MSG_SIZE];
        let mut rsp = [0u8; MAX_SPDM_MSG_SIZE];

        // Read the entire message into our address space.
        source
            .read_range(0..source.len(), &mut req)
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        let (reply, res) =
            self.responder.handle_msg(&req[..source.len()], &mut rsp);
        ringbuf_entry!(LogMsg::State(self.responder.state().into()));
        ringbuf_entry!(LogMsg::RspLen(reply.len()));

        // There was a protocol error. Just go back to the initial state.
        if res.is_err() {
            self.responder.reset();
            ringbuf_entry!(LogMsg::State(self.responder.state().into()));

            // A reply is always valid, even in the case of an error, since we want
            // to inform the requester of any SPDM errors that should go over the wire. In some
            // cases, reply may be of zero length however, indicating that nothing should be
            // sent to the responder and any connections should be closed. In this
            // case we return an Error.
            //
            // TODO: Map actual underlying errors?
            if reply.is_empty() {
                return Err(SpdmError::ResponderReset.into());
            }
        }

        self.valid = reply.len();

        sink.write_range(0..reply.len(), reply)
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        self.valid = 0; // Comsume the stored message

        Ok(reply.len())
    }
}

mod idl {
    use super::SpdmError;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
