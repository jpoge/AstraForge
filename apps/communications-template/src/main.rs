// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use communications_template::{
    CommunicationApp, CommunicationConfig, COMM_DOWNLINK_TOPIC, COMM_STATUS_TOPIC,
};
use fsw_sdk_app::MissionApp;
use fsw_sdk_core::MessageBus;
use fsw_sdk_template_launcher::run_host_sim_example;

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let platform = run_host_sim_example(
        || CommunicationApp::new(CommunicationConfig::default()),
        |app, ctx, _platform| {
            app.queue_downlink_frame(b"example_downlink_frame".to_vec());
            app.step(ctx)
        },
    )?;

    if let Some(status) = platform.bus().receive(&COMM_STATUS_TOPIC)? {
        println!("{}", String::from_utf8_lossy(&status));
    }
    if let Some(frame) = platform.bus().receive(&COMM_DOWNLINK_TOPIC)? {
        println!("downlink={}", String::from_utf8_lossy(&frame));
    }

    Ok(())
}
