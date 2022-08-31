// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use hubpack::serialize;
use ringbuf::*;
use sprockets_common::certificates::SerialNumber;
use sprockets_common::msgs::{RotError, RotResponseV1, RotResultV1};
use sprockets_common::random_buf;
use sprockets_rot::{RotConfig, RotSprocket};

#[derive(Copy, Clone, PartialEq)]
enum Trace {
    None,
    Line,
}
ringbuf!(Trace, 32, Trace::None);

pub fn init() -> RotSprocket {
    ringbuf_entry!(Trace::Line);
    let manufacturing_keypair = salty::Keypair::from(&random_buf());
    ringbuf_entry!(Trace::Line);
    let config = RotConfig::bootstrap_for_testing(
        &manufacturing_keypair,
        salty::Keypair::from(&random_buf()),
        SerialNumber(random_buf()),
    );
    ringbuf_entry!(Trace::Line);
    let r = RotSprocket::new(config);
    ringbuf_entry!(Trace::Line);
    r
}

// Serialize an Error response for a poorly encoded request
//
// Return the number of bytes written to `buf`.
pub fn bad_encoding_rsp(buf: &mut [u8]) -> usize {
    let rsp = RotResponseV1 {
        version: 1,
        id: 0,
        result: RotResultV1::Err(RotError::BadEncoding),
    };
    serialize(buf, &rsp).unwrap()
}
