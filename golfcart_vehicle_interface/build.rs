//! Build-time codegen: turn `CAX_ADS_CAN.dbc` into a typed Rust module.
//!
//! Output lands at `$OUT_DIR/dbc_messages.rs` and is `include!()`-ed by
//! `src/dbc.rs`. Re-run on any change to the DBC.

use std::{env, fs, io::Write, path::PathBuf};

const DBC_FILE: &str = "CAX_ADS_CAN.dbc";

fn main() -> anyhow::Result<()> {
    println!("cargo:rerun-if-changed={DBC_FILE}");
    println!("cargo:rerun-if-changed=build.rs");

    let mut dbc = fs::read_to_string(DBC_FILE)?;
    // dbc-codegen 0.3 panics-via-duplicate-variant when a `VAL_` table reuses
    // the same enum-name twice. CAX_ADS_CAN has `2 "Reserved" 3 "Reserved"` on
    // Ads_Vcu_Ads_Status — rename the second one so codegen produces a unique
    // variant. Both values still mean "do not use" per the protocol.
    dbc = dbc.replacen(
        r#"2 "Reserved" 3 "Reserved""#,
        r#"2 "Reserved" 3 "Reserved2""#,
        1,
    );

    let mut buf: Vec<u8> = Vec::new();
    dbc_codegen::codegen(DBC_FILE, dbc.as_bytes(), &mut buf, false)?;

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
    let out_path = out_dir.join("dbc_messages.rs");
    fs::File::create(&out_path)?.write_all(cleaned.as_bytes())?;
    Ok(())
}
