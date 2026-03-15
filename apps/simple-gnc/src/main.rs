// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_core::SdkError;
use simple_gnc::{run_mission, MissionConfig};

fn main() -> Result<(), SdkError> {
    let cli = parse_cli();
    let mut mission_config = MissionConfig::load(&cli.config_path)?;
    if let Some(rate_hz) = cli.rate_hz_override {
        mission_config.ekf_rate_hz = rate_hz;
    }

    let report = run_mission(&mission_config)?;
    if let Some(payload) = report.last_topic_payload {
        println!("{payload}");
    }
    println!("Loaded config from {}", cli.config_path);
    println!(
        "Final solution @ {} ms on {}: pos=({:.2}, {:.2}, {:.2}) m vel=({:.2}, {:.2}, {:.2}) m/s attitude=({:.3}, {:.3}, {:.3}) rad",
        report.final_solution.timestamp.0,
        report.platform,
        report.final_solution.position_m[0],
        report.final_solution.position_m[1],
        report.final_solution.position_m[2],
        report.final_solution.velocity_mps[0],
        report.final_solution.velocity_mps[1],
        report.final_solution.velocity_mps[2],
        report.final_solution.attitude_rad[0],
        report.final_solution.attitude_rad[1],
        report.final_solution.attitude_rad[2]
    );

    Ok(())
}

struct Cli {
    config_path: String,
    rate_hz_override: Option<u32>,
}

fn parse_cli() -> Cli {
    let mut cli = Cli {
        config_path: "apps/simple-gnc/config/mission.toml".to_string(),
        rate_hz_override: None,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--config" {
            if let Some(path) = args.next() {
                cli.config_path = path;
            }
        } else if arg == "--rate-hz" {
            if let Some(value) = args.next() {
                if let Ok(rate_hz) = value.parse::<u32>() {
                    cli.rate_hz_override = Some(rate_hz.max(1));
                }
            }
        }
    }
    cli
}
