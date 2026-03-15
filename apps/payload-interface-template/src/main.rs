// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_app::MissionApp;
use fsw_sdk_core::MessageBus;
use fsw_sdk_template_launcher::run_host_sim_example;
use payload_interface_template::{
    PayloadInterfaceApp, PAYLOAD_COMMAND_TOPIC, PAYLOAD_STATUS_TOPIC,
};

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let platform = run_host_sim_example(PayloadInterfaceApp::new, |app, ctx, platform| {
        platform
            .bus()
            .publish(&PAYLOAD_COMMAND_TOPIC, b"example_payload_command".to_vec())?;
        app.step(ctx)
    })?;

    if let Some(status) = platform.bus().receive(&PAYLOAD_STATUS_TOPIC)? {
        println!("{}", String::from_utf8_lossy(&status));
    }

    Ok(())
}
