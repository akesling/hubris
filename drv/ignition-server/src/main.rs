// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Server for interacting with Ignition Controllers.

#![no_std]
#![no_main]

use drv_ignition_api::*;
use drv_sidecar_mainboard_controller::ignition::*;
use ringbuf::*;
use userlib::*;

task_slot!(FPGA, fpga);
#[cfg(feature = "sequencer")]
task_slot!(SEQUENCER, sequencer);

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
enum Trace {
    None,
    AwaitingMainboardControllerReady,
    PortCount(u8),
    PresenceUpdate(u64),
    PresencePollError(IgnitionError),
}
ringbuf!(Trace, 16, Trace::None);

const TIMER_NOTIFICATION_MASK: u32 = 1 << 0;
const TIMER_INTERVAL: u64 = 1000;

#[export_name = "main"]
fn main() -> ! {
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl {
        controller: IgnitionController::new(FPGA.get_task_id()),
        port_count: 0,
        last_presence_summary: 0,
    };

    // This task is expected to run in an environment where a sequencer is
    // responsible for configuring the mainboard controller/Ignition Controller
    // logic. But for development purposes it may make sense for this assumption
    // not to be true and run this task with something else guaranteeing that
    // the mainboard controller (or something which looks like it) is present
    // and ready.
    #[cfg(feature = "sequencer")]
    {
        let sequencer =
            drv_sidecar_seq_api::Sequencer::from(SEQUENCER.get_task_id());

        // Poll the sequencer to determine if the mainboard controller is
        // ready.
        ringbuf_entry!(Trace::AwaitingMainboardControllerReady);
        while !sequencer.mainboard_controller_ready().unwrap_or(false) {
            hl::sleep_for(25);
        }
    }

    // Determine the number of Ignition controllers available.
    server.port_count = server.controller.port_count().unwrap_lite();
    ringbuf_entry!(Trace::PortCount(server.port_count));

    // Set a timer in the past causing the presence state to be polled and
    // updated as soon as the serving loop starts.
    sys_set_timer(Some(sys_get_timer().now), TIMER_NOTIFICATION_MASK);

    loop {
        idol_runtime::dispatch_n(&mut incoming, &mut server);
    }
}

struct ServerImpl {
    controller: IgnitionController,
    port_count: u8,
    last_presence_summary: u64,
}

impl ServerImpl {
    fn poll_presence(&mut self) -> Result<(), IgnitionError> {
        let current_presence_summary = self.controller.presence_summary()?;

        if current_presence_summary != self.last_presence_summary {
            ringbuf_entry!(Trace::PresenceUpdate(current_presence_summary));
            self.last_presence_summary = current_presence_summary;
        }

        Ok(())
    }

    fn port_state(&self, port: u8) -> Result<PortState, IgnitionError> {
        self.controller.state(port).map_err(IgnitionError::from)
    }
}

type RequestError = idol_runtime::RequestError<IgnitionError>;

impl idl::InOrderIgnitionImpl for ServerImpl {
    fn port_count(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<u8, RequestError> {
        if self.port_count == 0xff {
            Err(RequestError::from(IgnitionError::FpgaError))
        } else {
            Ok(self.port_count)
        }
    }

    fn presence_summary(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<u64, RequestError> {
        self.controller
            .presence_summary()
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)
    }

    fn state(
        &mut self,
        _: &userlib::RecvMessage,
        port: u8,
    ) -> Result<PortState, RequestError> {
        if port >= self.port_count {
            return Err(RequestError::from(IgnitionError::InvalidPort));
        }

        self.port_state(port).map_err(RequestError::from)
    }

    fn counters(
        &mut self,
        _: &userlib::RecvMessage,
        port: u8,
    ) -> Result<Counters, RequestError> {
        if port >= self.port_count {
            return Err(RequestError::from(IgnitionError::InvalidPort));
        }

        self.controller
            .counters(port)
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)
    }

    fn link_events(
        &mut self,
        _: &userlib::RecvMessage,
        port: u8,
        link: LinkSelect,
    ) -> Result<LinkEvents, RequestError> {
        if port >= self.port_count {
            return Err(RequestError::from(IgnitionError::InvalidPort));
        }

        self.controller
            .link_events(port, link)
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)
    }

    fn clear_link_events(
        &mut self,
        _: &userlib::RecvMessage,
        port: u8,
        link: LinkSelect,
    ) -> Result<(), RequestError> {
        if port >= self.port_count {
            return Err(RequestError::from(IgnitionError::InvalidPort));
        }

        self.controller
            .clear_link_events(port, link)
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)
    }

    fn send_request(
        &mut self,
        _: &userlib::RecvMessage,
        port: u8,
        request: Request,
    ) -> Result<(), RequestError> {
        if port >= self.port_count {
            return Err(RequestError::from(IgnitionError::InvalidPort));
        }

        let port_state = self.port_state(port).map_err(RequestError::from)?;
        let target_state = port_state.target().ok_or_else(|| {
            RequestError::from(IgnitionError::NoTargetPresent)
        })?;

        if target_state.system_power_reset_in_progress()
            || target_state.system_power_off_in_progress()
            || target_state.system_power_on_in_progress()
        {
            return Err(RequestError::from(IgnitionError::RequestInProgress));
        }

        self.controller
            .set_request(port, request)
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)
    }

    fn state_dump(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<[u64; PORT_MAX], RequestError> {
        let mut state = [0u64; PORT_MAX];
        let mut summary = self
            .controller
            .presence_summary()
            .map_err(IgnitionError::from)
            .map_err(RequestError::from)?;

        for i in 0..core::cmp::min(state.len(), self.port_count as usize) {
            // Check if the present bit is set in the summary.
            if summary & 0x1 != 0 {
                state[i] =
                    self.port_state(i as u8).map_err(RequestError::from)?.0;
            }
            // Advance to the next port.
            summary = summary >> 1;
        }

        Ok(state)
    }
}

impl idol_runtime::NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        TIMER_NOTIFICATION_MASK
    }

    fn handle_notification(&mut self, _bits: u32) {
        let start = sys_get_timer().now;

        // Only poll the presence summary if the port count seems reasonable. A
        // count of 0xff may occur if the FPGA is running an incorrect
        // bitstream.
        if self.port_count > 0 && self.port_count != 0xff {
            if let Err(e) = self.poll_presence() {
                ringbuf_entry!(Trace::PresencePollError(e));
            }
        }

        let finish = sys_get_timer().now;

        // We now know when we were notified and when any work was completed.
        // Note that the assumption here is that `start` < `finish` and that
        // this won't hold if the system time rolls over. But, the system timer
        // is a u64, with each bit representing a ms, so in practice this should
        // be fine. Anyway, armed with this information, find the next deadline
        // some multiple of `TIMER_INTERVAL` in the future.

        let delta = finish - start;
        let next_deadline = finish + TIMER_INTERVAL - (delta % TIMER_INTERVAL);

        sys_set_timer(Some(next_deadline), TIMER_NOTIFICATION_MASK);
    }
}

mod idl {
    use drv_ignition_api::*;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
