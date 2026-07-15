//! Build-time codegen: turn `CAX_ADS_CAN.dbc` into a typed Rust module
//! and a small physical-limit table.
//!
//! Outputs (both per-build, not committed):
//!   * `$OUT_DIR/dbc_messages.rs`  — `dbc-codegen` output, included by `src/dbc.rs`
//!   * `$OUT_DIR/dbc_limits.rs`    — derived min/max constants for setpoint
//!                                    clamping, included by `src/dbc.rs`
//!
//! The DBC file itself is proprietary and is gitignored. Locate it via:
//!   1. `CAX_ADS_DBC` env var (absolute path), or
//!   2. `CAX_ADS_CAN.dbc` in the crate root.

use anyhow::{anyhow, Context, Result};
use std::{env, fs, io::Write, path::PathBuf};

const DEFAULT_DBC: &str = "CAX_ADS_CAN.dbc";

fn main() -> Result<()> {
    println!("cargo:rerun-if-env-changed=CAX_ADS_DBC");
    println!("cargo:rerun-if-changed={DEFAULT_DBC}");
    println!("cargo:rerun-if-changed=build.rs");

    let dbc_path = locate_dbc()?;
    println!("cargo:rerun-if-changed={}", dbc_path.display());

    let mut dbc = fs::read_to_string(&dbc_path)
        .with_context(|| format!("reading DBC at {}", dbc_path.display()))?;

    // dbc-codegen 0.3 panics-via-duplicate-variant when a `VAL_` table reuses
    // the same enum-name twice. The vendor file has `2 "Reserved" 3 "Reserved"`
    // on Ads_Vcu_Ads_Status — rename the second one so codegen produces a
    // unique variant. Both values still mean "do not use" per the protocol.
    dbc = dbc.replacen(
        r#"2 "Reserved" 3 "Reserved""#,
        r#"2 "Reserved" 3 "Reserved2""#,
        1,
    );

    // Newer vendor DBC revisions prefix message names with the bus direction
    // (`RX_ADS_VCU_*` / `TX_VCU_ADS_*`, from the VCU's point of view). Strip
    // the prefixes so codegen keeps emitting the `AdsVcu*` / `VcuAds*` type
    // names the rest of the crate is written against. Only `BO_` lines carry
    // message names — CM_/VAL_/SG_ reference messages by numeric ID.
    dbc = dbc
        .lines()
        .map(|line| {
            if line.starts_with("BO_ ") {
                line.replacen(" RX_ADS_VCU_", " ADS_VCU_", 1)
                    .replacen(" TX_VCU_ADS_", " VCU_ADS_", 1)
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");

    let mut buf: Vec<u8> = Vec::new();
    dbc_codegen::codegen(
        dbc_path.file_name().unwrap_or_default().to_string_lossy().as_ref(),
        dbc.as_bytes(),
        &mut buf,
        false,
    )?;

    // Two transforms on the generator output:
    //  1. Strip leading `#![...]` inner attributes and `//!` doc comments —
    //     they're illegal inside `include!()`, which is how we splice the
    //     generated module into our crate.
    //  2. Replace `((value - offset) / factor) as iN/uN` with a rounded cast
    //     so e.g. 1.0/0.001 → 1000 instead of 999 (f32 rounding bites the
    //     bare `as` cast). dbc-codegen 0.3 always truncates; we want
    //     round-to-nearest for control-stack ergonomics.
    let raw = String::from_utf8(buf)?;
    let cleaned: String = raw
        .lines()
        .map(|line| {
            let trimmed = line.trim_start();
            if trimmed.starts_with("#![") || trimmed.starts_with("//!") {
                String::new()
            } else if let Some(idx) = line.find("((value - offset) / factor) as ") {
                let (head, tail) = line.split_at(idx);
                let tail = tail.replacen(
                    "((value - offset) / factor) as ",
                    "((value - offset) / factor).round() as ",
                    1,
                );
                format!("{head}{tail}")
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");

    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    fs::File::create(out_dir.join("dbc_messages.rs"))?.write_all(cleaned.as_bytes())?;

    let limits = parse_limits(&dbc)?;
    fs::File::create(out_dir.join("dbc_limits.rs"))?
        .write_all(emit_limits(&limits).as_bytes())?;

    Ok(())
}

/// Resolve where to read the DBC from. Prefer the env var so users can keep
/// the file out of the source tree entirely.
fn locate_dbc() -> Result<PathBuf> {
    if let Ok(p) = env::var("CAX_ADS_DBC") {
        let path = PathBuf::from(&p);
        if !path.exists() {
            return Err(anyhow!(
                "CAX_ADS_DBC points at '{p}' but the file does not exist"
            ));
        }
        return Ok(path);
    }
    let local = PathBuf::from(DEFAULT_DBC);
    if local.exists() {
        return Ok(local);
    }
    Err(anyhow!(
        "Vendor CAN database not found. \n\
         Obtain CAX_ADS_CAN.dbc from Turing Drive and either:\n  \
         1) place it at golfcart_vehicle_interface/{DEFAULT_DBC}, or\n  \
         2) set the CAX_ADS_DBC env var to its absolute path before building."
    ))
}

/// Physical-range bounds we want at runtime to clamp Autoware setpoints.
/// Parsed out of the DBC `SG_` definitions for the relevant TX signals so
/// the numbers (which are vendor IP) never appear in the source tree.
#[derive(Debug, Default)]
struct Limits {
    throttle_pct_min: Option<f64>,
    throttle_pct_max: Option<f64>,
    accel_mps2_min: Option<f64>,
    accel_mps2_max: Option<f64>,
    speed_mps_min: Option<f64>,
    speed_mps_max: Option<f64>,
    brake_pressure_mpa_min: Option<f64>,
    brake_pressure_mpa_max: Option<f64>,
    decel_mps2_min: Option<f64>,
    decel_mps2_max: Option<f64>,
    tire_angle_deg_min: Option<f64>,
    tire_angle_deg_max: Option<f64>,
}

fn parse_limits(dbc: &str) -> Result<Limits> {
    let mut out = Limits::default();
    for line in dbc.lines() {
        let line = line.trim_start();
        if !line.starts_with("SG_ ") {
            continue;
        }
        // Example signal: `SG_ Ads_Vcu_Target_Speed : 32|16@1- (0.001,0) [-32.768|32.767] "m/s"  VCU`
        let (lo, hi) = match extract_range(line) {
            Some(p) => p,
            None => continue,
        };
        let assign = match line.split_whitespace().nth(1) {
            Some(name) => name,
            None => continue,
        };
        match assign {
            "Ads_Vcu_Target_Throttle_Pos" => {
                out.throttle_pct_min = Some(lo);
                out.throttle_pct_max = Some(hi);
            }
            "Ads_Vcu_Target_Acceleration" => {
                out.accel_mps2_min = Some(lo);
                out.accel_mps2_max = Some(hi);
            }
            "Ads_Vcu_Target_Speed" => {
                out.speed_mps_min = Some(lo);
                out.speed_mps_max = Some(hi);
            }
            "Ads_Vcu_Target_Pressure" => {
                out.brake_pressure_mpa_min = Some(lo);
                out.brake_pressure_mpa_max = Some(hi);
            }
            "Ads_Vcu_Target_Deceleration" => {
                out.decel_mps2_min = Some(lo);
                out.decel_mps2_max = Some(hi);
            }
            "Ads_Vcu_Target_Tire_Angle" => {
                out.tire_angle_deg_min = Some(lo);
                out.tire_angle_deg_max = Some(hi);
            }
            _ => {}
        }
    }
    Ok(out)
}

fn extract_range(sg_line: &str) -> Option<(f64, f64)> {
    // Pull the substring inside `[ … | … ]`.
    let lb = sg_line.find('[')?;
    let rb = sg_line[lb..].find(']')? + lb;
    let inner = &sg_line[lb + 1..rb];
    let mut parts = inner.split('|');
    let lo = parts.next()?.trim().parse().ok()?;
    let hi = parts.next()?.trim().parse().ok()?;
    Some((lo, hi))
}

fn emit_limits(l: &Limits) -> String {
    let mut s = String::from(
        "// Auto-generated by build.rs from the Turing Drive DBC. \
         Numbers come from the vendor file and are NOT committed.\n",
    );
    for (name, value) in [
        ("THROTTLE_PCT_MIN", l.throttle_pct_min),
        ("THROTTLE_PCT_MAX", l.throttle_pct_max),
        ("ACCEL_MPS2_MIN", l.accel_mps2_min),
        ("ACCEL_MPS2_MAX", l.accel_mps2_max),
        ("SPEED_MPS_MIN", l.speed_mps_min),
        ("SPEED_MPS_MAX", l.speed_mps_max),
        ("BRAKE_PRESSURE_MPA_MIN", l.brake_pressure_mpa_min),
        ("BRAKE_PRESSURE_MPA_MAX", l.brake_pressure_mpa_max),
        ("DECEL_MPS2_MIN", l.decel_mps2_min),
        ("DECEL_MPS2_MAX", l.decel_mps2_max),
        ("TIRE_ANGLE_DEG_MIN", l.tire_angle_deg_min),
        ("TIRE_ANGLE_DEG_MAX", l.tire_angle_deg_max),
    ] {
        let v = value.expect(
            "DBC missing range for a Target_* signal — check vendor file integrity",
        );
        s.push_str(&format!(
            "#[allow(dead_code)] pub const {name}: f32 = {v}_f32;\n"
        ));
    }
    s
}
