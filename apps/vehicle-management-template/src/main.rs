// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_app::MissionApp;
use fsw_sdk_core::MessageBus;
use fsw_sdk_template_launcher::run_host_sim_example;
use vehicle_management_template::{VehicleEvent, VehicleManagementApp, VEHICLE_MODE_TOPIC};

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let platform = run_host_sim_example(VehicleManagementApp::new, |app, ctx, _platform| {
        app.queue_event(VehicleEvent::BootComplete);
        app.queue_event(VehicleEvent::EnterOperational);
        app.queue_event(VehicleEvent::EnterOperational);
        app.step(ctx)
    })?;

    if let Some(mode) = platform.bus().receive(&VEHICLE_MODE_TOPIC)? {
        println!("{}", String::from_utf8_lossy(&mode));
    }

    Ok(())
}
