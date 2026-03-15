// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_app::MissionApp;
use fsw_sdk_core::MessageBus;
use fsw_sdk_template_launcher::run_host_sim_example;
use thermal_management_template::{ThermalManagementApp, THERMAL_STATUS_TOPIC};

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let platform = run_host_sim_example(ThermalManagementApp::new, |app, ctx, _platform| {
        app.update_zone_temperature("avionics_bay", 32.5);
        app.update_zone_temperature("battery_pack", 12.0);
        app.step(ctx)
    })?;

    if let Some(status) = platform.bus().receive(&THERMAL_STATUS_TOPIC)? {
        println!("{}", String::from_utf8_lossy(&status));
    }

    Ok(())
}
