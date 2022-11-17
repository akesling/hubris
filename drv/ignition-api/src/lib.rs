// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! API crate for the Ignition controller.

#![no_std]

use derive_idol_err::IdolError;
use derive_more::{Display, From, LowerHex, UpperHex};
use drv_fpga_api::FpgaError;
use idol_runtime::ServerDeath;
use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::FromPrimitive;
use zerocopy::{AsBytes, FromBytes, Unaligned};

// The `presence_summary` vector (see `ignition-controller`) is implicitly
// capped at 40 bits by (the RTL of) the mainboard controller. This constant is
// used to conservatively allocate an array type which can contain the port
// state for all ports. The actual number of ports configured in the system can
// be learned through the `port_count()` function below.
pub const PORT_MAX: u8 = 40;

#[derive(
    Copy,
    Clone,
    Debug,
    PartialEq,
    Eq,
    From,
    FromPrimitive,
    ToPrimitive,
    IdolError,
)]
pub enum IgnitionError {
    ServerDied,
    FpgaError,
    InvalidPort,
    InvalidValue,
    NoTargetPresent,
    RequestInProgress,
}

impl From<ServerDeath> for IgnitionError {
    fn from(_e: ServerDeath) -> Self {
        Self::ServerDied
    }
}

impl From<FpgaError> for IgnitionError {
    fn from(_e: FpgaError) -> Self {
        Self::FpgaError
    }
}

pub struct Ignition {
    controller: idl::Ignition,
}

impl Ignition {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self {
            controller: idl::Ignition::from(task_id),
        }
    }

    pub fn port_count(&self) -> Result<u8, IgnitionError> {
        self.controller.port_count()
    }

    pub fn presence_summary(&self) -> Result<u64, IgnitionError> {
        self.controller.presence_summary()
    }

    pub fn port(&self, port: u8) -> Result<Port, IgnitionError> {
        self.controller.port_state(port).map(Port::from)
    }

    pub fn target(&self, port: u8) -> Result<Option<Target>, IgnitionError> {
        self.port(port).map(|p| p.target)
    }

    pub fn send_request(
        &self,
        port: u8,
        request: Request,
    ) -> Result<(), IgnitionError> {
        self.controller.send_request(port, request)
    }

    pub fn counters(&self, port: u8) -> Result<Counters, IgnitionError> {
        self.controller.counters(port)
    }

    pub fn link_events(
        &self,
        port: u8,
        txr: TransceiverSelect,
    ) -> Result<LinkEvents, IgnitionError> {
        self.controller.link_events(port, txr).map(LinkEvents::from)
    }

    pub fn clear_link_events(
        &self,
        port: u8,
        txr: TransceiverSelect,
    ) -> Result<(), IgnitionError> {
        self.controller.clear_link_events(port, txr)
    }

    pub fn targets(
        &self,
    ) -> Result<[Option<Target>; PORT_MAX as usize], IgnitionError> {
        let states = self.controller.port_state_batch(u64::MAX)?;
        let mut targets = [None; PORT_MAX as usize];

        for (state, maybe_target) in states.iter().zip(targets.iter_mut()) {
            *maybe_target = Port::from(*state).target;
        }

        Ok(targets)
    }
}

#[derive(
    Copy,
    Clone,
    Debug,
    Default,
    PartialEq,
    Eq,
    FromPrimitive,
    From,
    FromBytes,
    AsBytes,
)]
#[repr(C)]
pub struct PortState(pub u64);

#[derive(Copy, Clone, Debug, Default)]
pub struct Port {
    pub receiver_status: ReceiverStatus,
    pub target: Option<Target>,
}

impl From<PortState> for Port {
    fn from(state: PortState) -> Self {
        let target_present = state.0.as_bytes()
            [usize::from(Addr::CONTROLLER_STATUS)]
            & Reg::CONTROLLER_STATUS::TARGET_PRESENT
            != 0;

        Self {
            receiver_status: ReceiverStatus::from(
                state.0.as_bytes()[usize::from(Addr::CONTROLLER_LINK_STATUS)],
            ),
            target: if target_present {
                Some(Target::from(state))
            } else {
                None
            },
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ReceiverStatus {
    pub aligned: bool,
    pub locked: bool,
    pub polarity_inverted: bool,
}

impl From<u8> for ReceiverStatus {
    fn from(r: u8) -> ReceiverStatus {
        use Reg::CONTROLLER_LINK_STATUS::*;

        ReceiverStatus {
            aligned: r & RECEIVER_ALIGNED != 0,
            locked: r & RECEIVER_LOCKED != 0,
            polarity_inverted: r & POLARITY_INVERTED != 0,
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Target {
    pub id: SystemId,
    pub power_state: SystemPowerState,
    pub power_reset_in_progress: bool,
    pub faults: SystemFaults,
    pub controller0_present: bool,
    pub controller1_present: bool,
}

impl Target {
    #[inline]
    pub fn request_in_progress(&self) -> bool {
        self.power_reset_in_progress
            || self.power_state == SystemPowerState::PoweringOff
            || self.power_state == SystemPowerState::PoweringOn
    }
}

impl From<PortState> for Target {
    fn from(state: PortState) -> Self {
        use Reg::TARGET_REQUEST_STATUS::*;
        use Reg::TARGET_SYSTEM_STATUS::*;

        let system_status =
            state.0.as_bytes()[usize::from(Addr::TARGET_SYSTEM_STATUS)];
        let request_status =
            state.0.as_bytes()[usize::from(Addr::TARGET_REQUEST_STATUS)];

        Target {
            id: SystemId(
                state.0.as_bytes()[usize::from(Addr::TARGET_SYSTEM_TYPE)],
            ),
            power_state: SystemPowerState::from((
                system_status,
                request_status,
            )),
            power_reset_in_progress: request_status & SYSTEM_RESET_IN_PROGRESS
                != 0,
            faults: SystemFaults::from(
                state.0.as_bytes()[usize::from(Addr::TARGET_SYSTEM_FAULTS)],
            ),
            controller0_present: system_status & CONTROLLER0_DETECTED != 0,
            controller1_present: system_status & CONTROLLER1_DETECTED != 0,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Display, PartialEq, Eq)]
pub enum SystemPowerState {
    #[default]
    Off,
    On,
    Aborted,
    PoweringOff,
    PoweringOn,
}

impl From<(u8, u8)> for SystemPowerState {
    fn from(state: (u8, u8)) -> Self {
        use Reg::TARGET_REQUEST_STATUS::*;
        use Reg::TARGET_SYSTEM_STATUS::*;

        let (system_status, request_status) = state;

        if system_status & SYSTEM_POWER_ABORT != 0 {
            SystemPowerState::Aborted
        } else if request_status & POWER_ON_IN_PROGRESS != 0 {
            SystemPowerState::PoweringOn
        } else if request_status & POWER_OFF_IN_PROGRESS != 0 {
            SystemPowerState::PoweringOff
        } else if system_status & SYSTEM_POWER_ENABLED != 0 {
            SystemPowerState::On
        } else {
            SystemPowerState::Off
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SystemFaults {
    pub power_a3: bool,
    pub power_a2: bool,
    pub rot: bool,
    pub sp: bool,
    pub reserved1: bool,
    pub reserved2: bool,
}

impl From<u8> for SystemFaults {
    fn from(r: u8) -> Self {
        use Reg::TARGET_SYSTEM_FAULTS::*;

        Self {
            power_a3: r & POWER_FAULT_A3 != 0,
            power_a2: r & POWER_FAULT_A2 != 0,
            rot: r & ROT_FAULT != 0,
            sp: r & SP_FAULT != 0,
            reserved1: r & RESERVED1 != 0,
            reserved2: r & RESERVED2 != 0,
        }
    }
}

#[derive(
    Copy,
    Clone,
    Debug,
    Default,
    Display,
    PartialEq,
    Eq,
    From,
    FromBytes,
    AsBytes,
    UpperHex,
    LowerHex,
    Unaligned,
)]
#[repr(C)]
pub struct SystemId(pub u8);

#[derive(
    Copy, Clone, Debug, PartialEq, Eq, From, FromPrimitive, ToPrimitive, AsBytes,
)]
#[repr(u8)]
pub enum Request {
    SystemPowerOff = 1,
    SystemPowerOn = 2,
    SystemPowerReset = 3,
}

impl From<Request> for u8 {
    fn from(r: Request) -> Self {
        r as u8
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, AsBytes, FromBytes)]
#[repr(C)]
pub struct Counters {
    pub status_received: u8,
    pub hello_sent: u8,
    pub request_sent: u8,
    pub message_dropped: u8,
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct LinkEvents {
    pub encoding_error: bool,
    pub decoding_error: bool,
    pub ordered_set_invalid: bool,
    pub message_version_invalid: bool,
    pub message_type_invalid: bool,
    pub message_checksum_invalid: bool,
}

impl LinkEvents {
    pub const NONE: Self = Self::from_u8(0);
    pub const ALL: Self = Self::from_u8(0x3f);

    /// Implement as a const function to allow use above.
    const fn from_u8(r: u8) -> Self {
        use Reg::CONTROLLER_LINK_EVENTS_SUMMARY::*;

        Self {
            encoding_error: r & ENCODING_ERROR != 0,
            decoding_error: r & DECODING_ERROR != 0,
            ordered_set_invalid: r & ORDERED_SET_INVALID != 0,
            message_version_invalid: r & MESSAGE_VERSION_INVALID != 0,
            message_type_invalid: r & MESSAGE_TYPE_INVALID != 0,
            message_checksum_invalid: r & MESSAGE_CHECKSUM_INVALID != 0,
        }
    }
}

impl From<u8> for LinkEvents {
    fn from(r: u8) -> Self {
        Self::from_u8(r)
    }
}

/// Link events are observed by a transceiver, therefor each link between a
/// Controller and Target has two sets of `LinkEvents`. The Target notifies both
/// Controllers when events are observed by either of its transceivers. As a
/// result each Controller keeps track of three sets of link events; its own
/// tranceiver to the Target and both transceivers of the Target. When operating
/// on `LinkEvents` this enum is used to select between the different sets.
#[derive(
    Copy,
    Clone,
    Debug,
    Display,
    PartialEq,
    Eq,
    From,
    FromPrimitive,
    ToPrimitive,
    AsBytes,
)]
#[repr(u8)]
pub enum TransceiverSelect {
    Controller = 1,
    TargetLink0 = 2,
    TargetLink1 = 3,
}

impl TransceiverSelect {
    pub const ALL: [Self; 3] =
        [Self::Controller, Self::TargetLink0, Self::TargetLink1];
}

mod idl {
    use super::{
        Counters, IgnitionError, PortState, Request, TransceiverSelect,
    };
    use userlib::sys_send;

    include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
}

mod reg_map {
    include!(concat!(env!("OUT_DIR"), "/ignition_controller.rs"));

    impl From<Addr> for usize {
        fn from(addr: Addr) -> Self {
            addr as usize
        }
    }
}

pub use reg_map::Addr;
pub use reg_map::Reg;
