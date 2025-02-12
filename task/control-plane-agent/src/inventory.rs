// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use core::fmt::{self, Write};
use gateway_messages::{
    sp_impl::DeviceDescription, DeviceCapabilities, DevicePresence, SpComponent,
};
use task_validate_api::DEVICES as VALIDATE_DEVICES;
use task_validate_api::{Validate, ValidateError, ValidateOk};
use userlib::UnwrapLite;

userlib::task_slot!(VALIDATE, validate);

pub(crate) struct Inventory {
    validate_task: Validate,
}

impl Inventory {
    pub(crate) fn new() -> Self {
        let () = ASSERT_EACH_DEVICE_FITS_IN_ONE_PACKET;

        Self {
            validate_task: Validate::from(VALIDATE.get_task_id()),
        }
    }

    pub(crate) fn num_devices(&self) -> usize {
        OUR_DEVICES.len() + VALIDATE_DEVICES.len()
    }

    pub(crate) fn device_description(
        &self,
        index: usize,
    ) -> DeviceDescription<'static> {
        // If `index` is in `0..OUR_DEVICES.len()`, return that device directly;
        // otherwise, subtract `OUR_DEVICES.len()` to shift it into the range
        // `0..VALIDATE_DEVICES.len()` and ask `validate`.
        if index < OUR_DEVICES.len() {
            OUR_DEVICES[index]
        } else {
            let index = index - OUR_DEVICES.len();
            self.device_description_for_validate_device(index)
        }
    }

    fn device_description_for_validate_device(
        &self,
        index: usize,
    ) -> DeviceDescription<'static> {
        let device = &VALIDATE_DEVICES[index];

        let presence = match self.validate_task.validate_i2c(index) {
            Ok(ValidateOk::Present | ValidateOk::Validated) => {
                DevicePresence::Present
            }
            Ok(ValidateOk::Removed) | Err(ValidateError::NotPresent) => {
                DevicePresence::NotPresent
            }
            Err(ValidateError::BadValidation) => DevicePresence::Failed,
            Err(ValidateError::Unavailable | ValidateError::DeviceOff) => {
                DevicePresence::Unavailable
            }
            Err(ValidateError::DeviceTimeout) => DevicePresence::Timeout,
            Err(ValidateError::InvalidDevice | ValidateError::DeviceError) => {
                DevicePresence::Error
            }
        };

        // This format string is statically guaranteed to fit in `component`
        // based on our `max_num_devices` submodule below (which only contains
        // static assertions that ensure this format string will fit!).
        let mut component = FmtComponentId::default();
        write!(
            &mut component,
            "{}{}",
            SpComponent::GENERIC_DEVICE_PREFIX,
            index
        )
        .unwrap_lite();

        let mut capabilities = DeviceCapabilities::empty();
        if device.num_measurement_channels > 0 {
            capabilities |= DeviceCapabilities::HAS_MEASUREMENT_CHANNELS;
        }
        DeviceDescription {
            component: SpComponent { id: component.id },
            device: device.device,
            description: device.description,
            capabilities,
            presence,
        }
    }
}

#[derive(Default)]
struct FmtComponentId {
    pos: usize,
    id: [u8; SpComponent::MAX_ID_LENGTH],
}

impl fmt::Write for FmtComponentId {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let remaining = &mut self.id[self.pos..];
        if s.len() <= remaining.len() {
            remaining[..s.len()].copy_from_slice(s.as_bytes());
            self.pos += s.len();
            Ok(())
        } else {
            Err(fmt::Error)
        }
    }
}

// List of logical or high-level components that this task is responsible for
// (or at least responds to in terms of MGS requests for status / update, even
// if another task is actually responsible for lower-level details).
//
// TODO: Are our device names and descriptions good enough, or are there more
//       specific names we should use? This may be answered when we expand
//       DeviceDescription with any VPD / serial numbers.
const OUR_DEVICES: &[DeviceDescription<'static>] = &[
    // We always include "ourself" as a component; this is the component name
    // MGS uses to send SP image updates.
    DeviceDescription {
        component: SpComponent::SP_ITSELF,
        device: SpComponent::SP_ITSELF.const_as_str(),
        description: "Service Processor",
        capabilities: DeviceCapabilities::UPDATEABLE,
        presence: DevicePresence::Present,
    },
    // If we have the auxflash feature enabled, report the auxflash as a
    // component. We do not mark it as explicitly "updateable", even though it
    // is written as a part of the SP update process. Crucially, that is a part
    // of updating the `SP_ITSELF` component; the auxflash is not independently
    // updateable.
    #[cfg(feature = "auxflash")]
    DeviceDescription {
        component: SpComponent::SP_AUX_FLASH,
        device: SpComponent::SP_AUX_FLASH.const_as_str(),
        description: "Service Processor auxiliary flash",
        capabilities: DeviceCapabilities::empty(),
        presence: DevicePresence::Present,
    },
    // If we're building for gimlet, we always claim to have a host CPU.
    //
    // This is a lie on gimletlet (where we still build with the "gimlet"
    // feature), but a useful one in general.
    #[cfg(feature = "gimlet")]
    DeviceDescription {
        component: SpComponent::SP3_HOST_CPU,
        device: SpComponent::SP3_HOST_CPU.const_as_str(),
        description: "Gimlet SP3 host cpu",
        capabilities: DeviceCapabilities::HAS_SERIAL_CONSOLE,
        presence: DevicePresence::Present, // TODO: ok to assume always present?
    },
    // If we're building for gimlet, we always claim to have host boot flash.
    //
    // This is a lie on gimletlet (where we still build with the "gimlet"
    // feature), and a less useful one than the host CPU (since trying to access
    // the "host flash" will fail unless we have an adapter providing QSPI
    // flash).
    #[cfg(feature = "gimlet")]
    DeviceDescription {
        component: SpComponent::HOST_CPU_BOOT_FLASH,
        device: SpComponent::HOST_CPU_BOOT_FLASH.const_as_str(),
        description: "Gimlet host boot flash",
        capabilities: DeviceCapabilities::UPDATEABLE,
        presence: DevicePresence::Present, // TODO: ok to assume always present?
    },
];

// We use a generic component ID of `{prefix}{index}` for all of
// `VALIDATE_DEVICES`; here we statically assert the maximum number of devices
// we can use with this scheme. At the time of writing this comment, our ID
// width is 16 bytes and the prefix is 4 bytes, allowing up to 999_999_999_999
// devices to be listed.
//
// We tag this with `#[allow(dead_code)]` to prevent warnings about the contents
// of this module not being used; the static assertion _is_ still checked.
#[allow(dead_code)]
mod max_num_devices {
    use super::{SpComponent, VALIDATE_DEVICES};

    // How many bytes are available for digits of a device index in base 10?
    const DIGITS_AVAILABLE: usize =
        SpComponent::MAX_ID_LENGTH - SpComponent::GENERIC_DEVICE_PREFIX.len();

    // How many devices can we list given `DIGITS_AVAILABLE`?
    const MAX_NUM_DEVICES: u64 = const_exp10(DIGITS_AVAILABLE);

    // Statically assert that we have at most that many devices.
    static_assertions::const_assert!(
        VALIDATE_DEVICES.len() as u64 <= MAX_NUM_DEVICES
    );

    // Helper function: computes 10^n at compile time.
    const fn const_exp10(mut n: usize) -> u64 {
        let mut x = 1;
        while n > 0 {
            x *= 10;
            n -= 1;
        }
        x
    }
}

// We will spread the contents of `DEVICES` out over multiple packets to MGS;
// however, we do _not_ currently handle the case where a single `DEVICES` entry
// is too large to fit in a packet, even if it's the only device present in that
// packet. Therefore, we assert at compile time via all the machinery below that
// each entry of `DEVICES` is small enough that it will indeed fit in one packet
// after being packed into a TLV triple.
const ASSERT_EACH_DEVICE_FITS_IN_ONE_PACKET: () =
    assert_each_device_tlv_fits_in_one_packet();

const fn assert_device_tlv_fits_in_one_packet(
    device: &'static str,
    description: &'static str,
) {
    use gateway_messages::{tlv, SerializedSize, MIN_TRAILING_DATA_LEN};

    let encoded_len = tlv::tlv_len(
        gateway_messages::DeviceDescriptionHeader::MAX_SIZE
            + device.len()
            + description.len(),
    );

    if encoded_len > MIN_TRAILING_DATA_LEN {
        panic!(concat!(
            "The device details (device and description) of at least one ",
            "device in the current app.toml are too long to fit in a single ",
            "TLV triple to send to MGS. Current Rust restrictions prevent us ",
            "from being able to specific the specific device in this error ",
            "message. Change this panic to `panic!(\"{{}}\", description)` ",
            "and rebuild to see the description of the too-long device ",
            "instead."
        ))
    }
}

const fn assert_each_device_tlv_fits_in_one_packet() {
    // Check devices described by `validate`.
    let mut i = 0;
    loop {
        if i == VALIDATE_DEVICES.len() {
            break;
        }
        assert_device_tlv_fits_in_one_packet(
            VALIDATE_DEVICES[i].device,
            VALIDATE_DEVICES[i].description,
        );
        i += 1;
    }

    // Check devices described by us.
    let mut i = 0;
    loop {
        if i == OUR_DEVICES.len() {
            break;
        }
        assert_device_tlv_fits_in_one_packet(
            OUR_DEVICES[i].device,
            OUR_DEVICES[i].description,
        );
        i += 1;
    }
}
