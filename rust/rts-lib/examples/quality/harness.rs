//! Scorecard types, scoring, JSON, and the table / diff / sweep renderers.
//!
//! The harness never asserts and never fails a build.
//! Invariants belong in the crate's `#[cfg(test)]` modules; here the same
//! quantities are counts feeding a score, so one already at zero keeps being
//! watched while the behaviour around it moves.

use std::collections::BTreeMap;
use std::path::PathBuf;

use serde::{Deserialize, Serialize};

/// One scored quantity: what it is, and the two values that pin its scale.
/// Direction is read off the anchors rather than stored: `good` is the
/// 100-point end, so `good < bad` *is* "lower is better".
#[derive(Clone, Copy, Debug)]
pub struct Metric {
    pub name: &'static str,
    pub unit: &'static str,
    /// Value scoring 100 points.
    pub good: f64,
    /// Value scoring 0 points.
    pub bad: f64,
    /// Share of the scenario score. 0 reports the metric without scoring it.
    pub weight: f64,
}

/// Declare a metric: `good` scores 100, `bad` scores 0, and `weight` is its
/// share of the scenario score (0 reports it without scoring it). Scenarios
/// import this as `m` to keep the anchor table one aligned line per metric.
pub const fn metric(
    name: &'static str,
    unit: &'static str,
    good: f64,
    bad: f64,
    weight: f64,
) -> Metric {
    Metric {
        name,
        unit,
        good,
        bad,
        weight,
    }
}

impl Metric {
    /// Linear interpolation between the anchors, clamped to 0..100.
    pub fn points(&self, value: f64) -> f64 {
        if !value.is_finite() {
            return 0.0;
        }
        let span = self.good - self.bad;
        if span == 0.0 {
            return if value == self.good { 100.0 } else { 0.0 };
        }
        let t = ((value - self.bad) / span).clamp(0.0, 1.0);
        // Explicit zero: `(bad - bad) / (good - bad)` renders as "-0".
        if t <= 0.0 { 0.0 } else { t * 100.0 }
    }

    /// This metric read at `value`.
    pub fn at(self, value: f64) -> Reading {
        Reading {
            metric: self,
            value,
        }
    }

    /// This metric read at `value`, or pinned to the zero-point anchor when
    /// the probe produced none: "no unit arrived, so no detour to average" is
    /// a bad outcome, not a missing measurement.
    pub fn or_bad(self, value: Option<f64>) -> Reading {
        self.at(value.unwrap_or(self.bad))
    }
}

pub struct Reading {
    pub metric: Metric,
    pub value: f64,
}

/// What a scenario is handed when it runs.
pub struct Ctx {
    /// Where reference fields are cached (`target/quality`).
    pub cache_dir: PathBuf,
    /// Where traces are written.
    pub out_dir: PathBuf,
    /// Whether this scenario should dump a per-tick trace.
    pub trace: bool,
}

pub struct ScenarioSpec {
    pub name: &'static str,
    pub run: fn(&Ctx) -> Vec<Reading>,
}

// ── Scorecard ────────────────────────────────────────────────────────────────

/// Bumped when the JSON shape changes incompatibly.
pub const FORMAT: u32 = 1;

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MetricResult {
    pub name: String,
    pub unit: String,
    pub value: f64,
    pub points: f64,
    pub weight: f64,
    pub good: f64,
    pub bad: f64,
}

impl MetricResult {
    fn anchors_match(&self, other: &MetricResult) -> bool {
        self.good == other.good && self.bad == other.bad && self.weight == other.weight
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ScenarioResult {
    pub name: String,
    pub score: f64,
    pub metrics: Vec<MetricResult>,
}

impl ScenarioResult {
    pub fn new(name: &str, readings: Vec<Reading>) -> ScenarioResult {
        let metrics: Vec<MetricResult> = readings
            .into_iter()
            .map(|r| MetricResult {
                name: r.metric.name.to_string(),
                unit: r.metric.unit.to_string(),
                value: r.value,
                points: r.metric.points(r.value),
                weight: r.metric.weight,
                good: r.metric.good,
                bad: r.metric.bad,
            })
            .collect();
        // Weighted mean over the scored metrics; weight-0 ones are diagnostics.
        let wsum: f64 = metrics.iter().map(|m| m.weight).sum();
        let score = if wsum > 0.0 {
            metrics.iter().map(|m| m.points * m.weight).sum::<f64>() / wsum
        } else {
            0.0
        };
        ScenarioResult {
            name: name.to_string(),
            score,
            metrics,
        }
    }

    fn metric(&self, name: &str) -> Option<&MetricResult> {
        self.metrics.iter().find(|m| m.name == name)
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Scorecard {
    pub format: u32,
    pub git_sha: String,
    pub git_dirty: bool,
    /// Digest over every anchor and weight; differing digests mean the two
    /// cards were scored on different scales.
    pub anchors: String,
    /// Tunables overridden for this run (`--sweep`); empty for a plain run.
    #[serde(default)]
    pub tuning: BTreeMap<String, f64>,
    pub scenarios: Vec<ScenarioResult>,
    /// Unweighted mean over scenarios, so one cannot dominate by declaring
    /// more metrics.
    pub total: f64,
}

impl Scorecard {
    pub fn new(scenarios: Vec<ScenarioResult>, tuning: BTreeMap<String, f64>) -> Scorecard {
        let total = if scenarios.is_empty() {
            0.0
        } else {
            scenarios.iter().map(|s| s.score).sum::<f64>() / scenarios.len() as f64
        };
        let (git_sha, git_dirty) = git_state();
        let anchors = anchor_digest(&scenarios);
        Scorecard {
            format: FORMAT,
            git_sha,
            git_dirty,
            anchors,
            tuning,
            scenarios,
            total,
        }
    }

    fn scenario(&self, name: &str) -> Option<&ScenarioResult> {
        self.scenarios.iter().find(|s| s.name == name)
    }
}

fn anchor_digest(scenarios: &[ScenarioResult]) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |bytes: &[u8]| {
        for &b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    for s in scenarios {
        mix(s.name.as_bytes());
        for m in &s.metrics {
            mix(m.name.as_bytes());
            mix(&m.good.to_le_bytes());
            mix(&m.bad.to_le_bytes());
            mix(&m.weight.to_le_bytes());
        }
    }
    format!("{h:016x}")
}

fn git_state() -> (String, bool) {
    let run = |args: &[&str]| -> Option<String> {
        let out = std::process::Command::new("git")
            .args(args)
            .current_dir(env!("CARGO_MANIFEST_DIR"))
            .output()
            .ok()?;
        out.status
            .success()
            .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
    };
    let sha = run(&["rev-parse", "--short", "HEAD"]).unwrap_or_else(|| "unknown".into());
    let dirty = run(&["status", "--porcelain", "--untracked-files=no"])
        .map(|s| !s.is_empty())
        .unwrap_or(false);
    (sha, dirty)
}

// ── Rendering ────────────────────────────────────────────────────────────────

const NAME_W: usize = 30;

/// Value formatting that keeps a column readable across the four orders of
/// magnitude these metrics span (ratios near 1, counts, ticks, pixels).
pub fn fmt_val(v: f64) -> String {
    if !v.is_finite() {
        return "  n/a".into();
    }
    let a = v.abs();
    if a >= 1000.0 {
        format!("{v:.0}")
    } else if a >= 100.0 {
        format!("{v:.1}")
    } else if a >= 10.0 {
        format!("{v:.2}")
    } else {
        format!("{v:.3}")
    }
}

fn fmt_delta(d: f64) -> String {
    if !d.is_finite() {
        return "".into();
    }
    if d == 0.0 {
        return "  ·".into();
    }
    let s = fmt_val(d.abs());
    format!("{}{}", if d > 0.0 { '+' } else { '-' }, s)
}

/// The run table: one block per scenario, its score and delta on the header
/// row and its raw metric columns underneath. The composite answers "did this
/// help overall", the columns "what did it trade away" — and these do trade:
/// tighter separation costs throughput, cohesion costs arrival time.
pub fn render(card: &Scorecard, base: Option<&Scorecard>) -> String {
    let mut out = String::new();
    let mut mismatched: Vec<String> = Vec::new();

    out.push_str(&format!(
        "{:<w$} {:>9} {:>6} {:>5}  {:>9} {:>7}\n",
        "scenario / metric",
        "value",
        "pts",
        "w",
        "Δ value",
        "Δ pts",
        w = NAME_W
    ));
    out.push_str(&format!("{}\n", "─".repeat(NAME_W + 40)));

    for s in &card.scenarios {
        let b = base.and_then(|b| b.scenario(&s.name));
        let dscore = b.map(|b| s.score - b.score);
        out.push_str(&format!(
            "{:<w$} {:>9} {:>6.1} {:>5}  {:>9} {:>7}\n",
            s.name,
            "",
            s.score,
            "",
            "",
            dscore.map(fmt_delta).unwrap_or_default(),
            w = NAME_W
        ));
        for m in &s.metrics {
            let bm = b.and_then(|b| b.metric(&m.name));
            let anchors_moved = bm.is_some_and(|bm| !m.anchors_match(bm));
            if anchors_moved {
                mismatched.push(format!("{}/{}", s.name, m.name));
            }
            let (dv, dp) = match bm {
                Some(bm) if !anchors_moved => (
                    fmt_delta(m.value - bm.value),
                    fmt_delta(m.points - bm.points),
                ),
                Some(_) => ("!".into(), "!".into()),
                None => (String::new(), String::new()),
            };
            let label = if m.weight == 0.0 {
                format!("  {} ·", m.name)
            } else {
                format!("  {}", m.name)
            };
            // No scale declared (good == bad) means a raw reading, not a
            // score. `·` already means "no change" in the delta columns, so it
            // cannot double as "no points" here.
            let pts = if m.good == m.bad {
                "n/a".to_string()
            } else {
                format!("{:.0}", m.points)
            };
            out.push_str(&format!(
                "{:<w$} {:>9} {:>6} {:>5.1}  {:>9} {:>7}\n",
                label,
                fmt_val(m.value),
                pts,
                m.weight,
                dv,
                dp,
                w = NAME_W
            ));
        }
    }

    out.push_str(&format!("{}\n", "─".repeat(NAME_W + 40)));
    // Same scenario set only: the total is a mean over scenarios, so a
    // `--only` run against a full baseline reports the skips as a regression.
    let same_set = base.is_some_and(|b| {
        b.scenarios.len() == card.scenarios.len()
            && card.scenarios.iter().all(|s| b.scenario(&s.name).is_some())
    });
    let dtotal = base.filter(|_| same_set).map(|b| card.total - b.total);
    out.push_str(&format!(
        "{:<w$} {:>9} {:>6.1} {:>5}  {:>9} {:>7}\n",
        "total",
        "",
        card.total,
        "",
        "",
        dtotal.map(fmt_delta).unwrap_or_default(),
        w = NAME_W
    ));

    // The digest covers only this card's scenarios, so a filtered run differs
    // by construction; the per-metric check above decides comparability.
    out.push_str(&format!(
        "\n{} {}anchors {}\n",
        card.git_sha,
        if card.git_dirty { "(dirty) " } else { "" },
        card.anchors,
    ));
    if !card.tuning.is_empty() {
        let cfg: Vec<String> = card
            .tuning
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect();
        out.push_str(&format!("tuning  {}\n", cfg.join(" ")));
    }
    if let Some(b) = base {
        out.push_str(&format!(
            "base    {} {}anchors {}\n",
            b.git_sha,
            if b.git_dirty { "(dirty) " } else { "" },
            b.anchors
        ));
        if !mismatched.is_empty() {
            out.push_str(&format!(
                "\nanchors changed on {} metric(s); those scores are not comparable:\n  {}\n",
                mismatched.len(),
                mismatched.join(", ")
            ));
        }
        let missing: Vec<&str> = b
            .scenarios
            .iter()
            .map(|s| s.name.as_str())
            .filter(|n| card.scenario(n).is_none())
            .collect();
        if !missing.is_empty() {
            out.push_str(&format!(
                "not run this time: {}\n(no total delta: the total is a mean over scenarios)\n",
                missing.join(", ")
            ));
        }
    }
    // Weight-0 rows are diagnostics: printed, never folded into a score.
    if card
        .scenarios
        .iter()
        .any(|s| s.metrics.iter().any(|m| m.weight == 0.0))
    {
        out.push_str("n/a in the pts column = reported, not scored\n");
    }
    out
}

/// One column of scenario scores per sweep configuration.
pub fn render_sweep(cards: &[(String, Scorecard)]) -> String {
    let mut out = String::new();
    let mut names: Vec<&str> = Vec::new();
    for (_, c) in cards {
        for s in &c.scenarios {
            if !names.contains(&s.name.as_str()) {
                names.push(&s.name);
            }
        }
    }
    out.push_str(&format!("{:<w$}", "scenario", w = NAME_W));
    for (label, _) in cards {
        out.push_str(&format!(" {label:>12}"));
    }
    out.push('\n');
    out.push_str(&format!(
        "{}\n",
        "─".repeat(NAME_W + 13 * cards.len().max(1))
    ));
    for name in names {
        out.push_str(&format!("{name:<NAME_W$}"));
        for (_, c) in cards {
            match c.scenario(name) {
                Some(s) => out.push_str(&format!(" {:>12.1}", s.score)),
                None => out.push_str(&format!(" {:>12}", "")),
            }
        }
        out.push('\n');
    }
    out.push_str(&format!(
        "{}\n",
        "─".repeat(NAME_W + 13 * cards.len().max(1))
    ));
    out.push_str(&format!("{:<NAME_W$}", "total"));
    for (_, c) in cards {
        out.push_str(&format!(" {:>12.1}", c.total));
    }
    out.push('\n');
    out
}

pub fn write_json(path: &PathBuf, card: &Scorecard) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let json = serde_json::to_string_pretty(card).expect("scorecard serialises");
    std::fs::write(path, json + "\n")
}

pub fn read_json(path: &PathBuf) -> Result<Scorecard, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let card: Scorecard =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if card.format != FORMAT {
        return Err(format!(
            "{}: scorecard format {}, this build writes {FORMAT}",
            path.display(),
            card.format
        ));
    }
    Ok(card)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_points_interpolate_in_both_directions() {
        let lower = metric("detour", "ratio", 1.0, 2.0, 1.0); // lower is better
        assert_eq!(lower.points(1.0), 100.0);
        assert_eq!(lower.points(2.0), 0.0);
        assert_eq!(lower.points(1.5), 50.0);
        let higher = metric("arrival", "frac", 1.0, 0.0, 1.0); // higher is better
        assert_eq!(higher.points(1.0), 100.0);
        assert_eq!(higher.points(0.0), 0.0);
        assert_eq!(higher.points(0.25), 25.0);
    }

    #[test]
    fn test_points_clamp_and_degenerate_cases() {
        let m = metric("detour", "ratio", 1.0, 2.0, 1.0);
        assert_eq!(m.points(0.1), 100.0, "better than `good` is still 100");
        assert_eq!(m.points(9.9), 0.0, "worse than `bad` is still 0");
        // Zero, not negative zero: the points column must not print "-0".
        assert!(m.points(2.0).is_sign_positive());
        assert_eq!(m.points(f64::NAN), 0.0);
        assert_eq!(m.points(f64::INFINITY), 0.0);
        // No scale declared: only the exact value scores.
        let flat = metric("cohesion", "radii", 0.0, 0.0, 0.0);
        assert_eq!(flat.points(0.0), 100.0);
        assert_eq!(flat.points(1.0), 0.0);
    }

    #[test]
    fn test_or_bad_pins_a_missing_reading_to_zero_points() {
        let m = metric("detour", "ratio", 1.0, 2.0, 1.0);
        assert_eq!(m.or_bad(None).value, 2.0);
        assert_eq!(m.points(m.or_bad(None).value), 0.0);
        assert_eq!(m.or_bad(Some(1.5)).value, 1.5);
    }

    #[test]
    fn test_scenario_score_is_weighted_and_ignores_diagnostics() {
        let s = ScenarioResult::new(
            "x",
            vec![
                metric("a", "", 1.0, 0.0, 3.0).at(1.0), // 100 pts, weight 3
                metric("b", "", 1.0, 0.0, 1.0).at(0.0), // 0 pts, weight 1
                metric("c", "", 1.0, 0.0, 0.0).at(0.0), // reported, unscored
            ],
        );
        assert_eq!(s.score, 75.0);
        assert_eq!(s.metrics.len(), 3, "diagnostics are still reported");
    }

    #[test]
    fn test_total_is_unweighted_across_scenarios() {
        // Four metrics in one scenario must not outvote one in another.
        let big = ScenarioResult::new(
            "big",
            (0..4)
                .map(|_| metric("m", "", 1.0, 0.0, 1.0).at(1.0))
                .collect(),
        );
        let small = ScenarioResult::new("small", vec![metric("m", "", 1.0, 0.0, 1.0).at(0.0)]);
        let card = Scorecard::new(vec![big, small], BTreeMap::new());
        assert_eq!(card.total, 50.0);
    }

    fn card_of(name: &str, good: f64, value: f64) -> Scorecard {
        Scorecard::new(
            vec![ScenarioResult::new(
                name,
                vec![metric("m", "ratio", good, 2.0, 1.0).at(value)],
            )],
            BTreeMap::new(),
        )
    }

    #[test]
    fn test_anchor_digest_tracks_anchors_not_values() {
        assert_eq!(
            card_of("s", 1.0, 1.5).anchors,
            card_of("s", 1.0, 1.9).anchors,
            "a different measurement is not a different scale"
        );
        assert_ne!(
            card_of("s", 1.0, 1.5).anchors,
            card_of("s", 1.2, 1.5).anchors,
            "a moved anchor must be visible in the digest"
        );
        assert_ne!(card_of("s", 1.0, 1.5).anchors, card_of("t", 1.0, 1.5).anchors);
    }

    #[test]
    fn test_render_flags_moved_anchors_instead_of_diffing_across_them() {
        let base = card_of("s", 1.0, 1.5);
        let moved = card_of("s", 1.2, 1.5);
        let out = render(&moved, Some(&base));
        assert!(out.contains("anchors changed"), "{out}");
        assert!(
            !out.contains("  ·  "),
            "a delta must not be printed across a moved anchor: {out}"
        );
        // Same anchors: the delta is printed normally.
        let same = render(&card_of("s", 1.0, 1.9), Some(&base));
        assert!(same.contains("+0.400"), "{same}");
        assert!(!same.contains("anchors changed"), "{same}");
    }

    #[test]
    fn test_no_total_delta_when_the_scenario_set_differs() {
        let base = Scorecard::new(
            vec![
                ScenarioResult::new("a", vec![metric("m", "", 1.0, 0.0, 1.0).at(1.0)]),
                ScenarioResult::new("b", vec![metric("m", "", 1.0, 0.0, 1.0).at(0.0)]),
            ],
            BTreeMap::new(),
        );
        // `a` alone totals 100 against a baseline of 50: an "improvement"
        // that is only the other scenario not running.
        let only_a = ScenarioResult::new("a", vec![metric("m", "", 1.0, 0.0, 1.0).at(1.0)]);
        let filtered = Scorecard::new(vec![only_a], BTreeMap::new());
        let out = render(&filtered, Some(&base));
        assert!(out.contains("no total delta"), "{out}");
        assert!(!out.contains("+50"), "{out}");
    }

    #[test]
    fn test_render_names_scenarios_the_base_had_and_this_run_skipped() {
        let base = Scorecard::new(
            vec![
                ScenarioResult::new("a", vec![metric("m", "", 1.0, 0.0, 1.0).at(1.0)]),
                ScenarioResult::new("b", vec![metric("m", "", 1.0, 0.0, 1.0).at(1.0)]),
            ],
            BTreeMap::new(),
        );
        let only_a = card_of("a", 1.0, 1.5);
        assert!(render(&only_a, Some(&base)).contains("not run this time: b"));
    }

    #[test]
    fn test_scorecard_json_round_trips_bit_for_bit() {
        // Full-mantissa, not tidy: `serde_json`'s default float parser is not
        // correctly rounded, which turned every metric into a ~1e-13 phantom
        // delta. `float_roundtrip` fixes it; this is the guard.
        let card = card_of("s", 1.0, 1.0 / 3.0);
        let text = serde_json::to_string_pretty(&card).unwrap();
        let back: Scorecard = serde_json::from_str(&text).unwrap();
        assert_eq!(back.anchors, card.anchors);
        assert_eq!(back.total.to_bits(), card.total.to_bits());
        let (a, b) = (&back.scenarios[0].metrics[0], &card.scenarios[0].metrics[0]);
        assert_eq!(a.value.to_bits(), b.value.to_bits());
        assert_eq!(a.points.to_bits(), b.points.to_bits());
    }

    #[test]
    fn test_a_rerun_against_its_own_card_shows_no_deltas() {
        // The scorecard's premise: the sim is deterministic, so an unchanged
        // run must diff to nothing. A float that survives scoring but not the
        // JSON shows up here as a stray delta.
        let card = card_of("s", 1.0, 1.0 / 3.0);
        let text = serde_json::to_string_pretty(&card).unwrap();
        let reloaded: Scorecard = serde_json::from_str(&text).unwrap();
        let out = render(&card, Some(&reloaded));
        assert!(
            !out.contains('+') && !out.contains('-'),
            "an unchanged rerun must print no deltas:\n{out}"
        );
    }
}
