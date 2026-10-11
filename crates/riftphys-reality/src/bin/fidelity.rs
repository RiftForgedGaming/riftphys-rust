//! Physics fidelity report: every closed-form benchmark at 60 Hz and 240 Hz.
//!
//!     riftphys-fidelity [--hz 60,240] [--json report.json]

use riftphys_reality::analytic::run_all;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let get = |name: &str| args.windows(2).find(|w| w[0] == name).map(|w| w[1].clone());
    let rates: Vec<u32> = get("--hz").unwrap_or_else(|| "60,240".into()).split(',').map(|s| s.trim().parse().expect("--hz")).collect();
    let mut json = Vec::new();
    let mut failed = 0;
    for hz in rates {
        let cases = run_all(1.0 / hz as f32);
        let passed = cases.iter().filter(|c| c.pass()).count();
        println!("\n=== {} Hz: {}/{} within tolerance ===", hz, passed, cases.len());
        println!("{:<4} {:<22} {:>12} {:>12} {:>12} {:>10}  {}", "", "case", "sim", "truth", "error", "tol", "quantity");
        for c in &cases {
            println!("{:<4} {:<22} {:>12.4} {:>12.4} {:>12} {:>10}  {}", if c.pass() { "ok" } else { "FAIL" },
                     c.name, c.sim, c.truth, c.error_text(), c.tol_text(), c.quantity);
            json.push(serde_json::json!({"hz": hz, "name": c.name, "quantity": c.quantity, "unit": c.unit,
                                         "sim": c.sim, "truth": c.truth, "error": c.error(), "tol": c.tol_text(), "pass": c.pass()}));
        }
        failed += cases.len() - passed;
    }
    if let Some(path) = get("--json") {
        std::fs::write(&path, serde_json::to_string_pretty(&json).unwrap()).expect("write report");
        println!("\nwrote {}", path);
    }
    if failed > 0 { println!("\n{} case(s) outside tolerance", failed); }
}
