//! Movement quality scorecard.
//!
//! A second benchmark family alongside `benches/{astar,delaunay,sim}.rs`:
//! instead of timing a tick, it scores *how well units path*, so a movement
//! change can be judged by numbers rather than by watching the Godot view.
//!
//! ```text
//! cargo run --release --example quality                   # run, diff vs baseline
//! cargo run --release --example quality -- --out a.json    # run, write a scorecard
//! cargo run --release --example quality -- --diff a.json b.json
//! cargo run --release --example quality -- --bless         # overwrite the baseline
//! cargo run --release --example quality -- --only door_funnel_200,counterflow_2x60
//! cargo run --release --example quality -- --trace door_funnel_200
//! cargo run --release --example quality -- --trace all
//! cargo run --release --example quality -- --sweep cohesion_gain=0.02:0.10:0.02
//! ```
//!
//! `--trace` writes `target/quality/trace_<scenario>.json`, played back by the
//! Godot scene `src/game/quality_replay.tscn`. Walls are timed events from the
//! live navmesh, so mid-run obstacle edits replay correctly. One JSON object
//! per unit per tick, so size scales with `units * ticks`: `--trace all` is
//! well over a hundred megabytes, and each file's size is printed. Derived and
//! large, so traces stay under `target/` and are never committed.
//!
//! Always `--release`: identical behaviour in debug, but tens of thousands of
//! ticks make it a coffee break.
//!
//! Nothing here reports wall-clock time, deliberately, so a slow-but-better
//! change scores as better. Timing is what `benches/` is for.
//!
//! The sim is deterministic, so every number is bit-identical run to run on the
//! same binary. No sampling, no noise floor: **any nonzero delta is a real
//! behaviour change**. Scorecards therefore compare across commits on one
//! machine, not across machines.

mod harness;
mod maps;
mod metrics;
mod reference;
mod scenarios;

use std::collections::BTreeMap;
use std::path::PathBuf;

use harness::{Ctx, Scorecard, ScenarioResult};

const USAGE: &str = "\
quality: movement quality scorecard

  (no args)                 run every scenario, diff against the baseline
  --out <file>              write the scorecard as JSON
  --diff <base> <new>       render two saved scorecards against each other
  --bless                   run, then overwrite quality_baseline.json
  --only <a,b,...>          run just these scenarios
  --trace <a,b,...|all>     also dump those scenarios' per-tick traces
                            (`all` traces every scenario that runs; the
                            path-level ones step no sim and produce none)
  --out-dir <dir>           where traces and reference caches go
                            (default: target/quality)
  --sweep <name=lo:hi:step> re-run the grid once per tunable value
                            (names are `sim::set_tuning`'s, e.g. cohesion_gain)
  --list                    print the scenario names
  -h, --help                this
";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match Cli::parse(&args) {
        Ok(cli) => cli.run(),
        Err(e) => {
            eprintln!("{e}\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}

/// Repo root: the committed baseline's home and `target/`'s parent.
/// Canonicalised so printed paths are the ones a reader would type.
fn root() -> PathBuf {
    let raw = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/.."));
    raw.canonicalize().unwrap_or(raw)
}

struct Cli {
    out: Option<PathBuf>,
    diff: Option<(PathBuf, PathBuf)>,
    bless: bool,
    only: Option<Vec<String>>,
    trace: Trace,
    out_dir: PathBuf,
    sweep: Option<Sweep>,
    list: bool,
}

/// Which scenarios should dump a per-tick trace.
enum Trace {
    None,
    All,
    Named(Vec<String>),
}

impl Trace {
    fn wants(&self, scenario: &str) -> bool {
        match self {
            Trace::None => false,
            Trace::All => true,
            Trace::Named(names) => names.iter().any(|n| n == scenario),
        }
    }

    /// The names to validate against the registry; `all` names nothing.
    fn names(&self) -> impl Iterator<Item = &String> {
        match self {
            Trace::Named(names) => names.iter(),
            _ => [].iter(),
        }
    }
}

struct Sweep {
    name: String,
    lo: f32,
    hi: f32,
    step: f32,
}

impl Cli {
    fn parse(args: &[String]) -> Result<Cli, String> {
        let mut cli = Cli {
            out: None,
            diff: None,
            bless: false,
            only: None,
            trace: Trace::None,
            out_dir: root().join("target/quality"),
            sweep: None,
            list: false,
        };
        let mut i = 0;
        let next = |i: &mut usize, flag: &str| -> Result<String, String> {
            *i += 1;
            args.get(*i)
                .cloned()
                .ok_or_else(|| format!("{flag} needs a value"))
        };
        while i < args.len() {
            match args[i].as_str() {
                "-h" | "--help" => {
                    print!("{USAGE}");
                    std::process::exit(0);
                }
                "--list" => cli.list = true,
                "--bless" => cli.bless = true,
                "--out" => cli.out = Some(PathBuf::from(next(&mut i, "--out")?)),
                "--out-dir" => cli.out_dir = PathBuf::from(next(&mut i, "--out-dir")?),
                "--trace" => {
                    let arg = next(&mut i, "--trace")?;
                    cli.trace = if arg == "all" {
                        Trace::All
                    } else {
                        Trace::Named(csv(&arg))
                    };
                }
                "--only" => cli.only = Some(csv(&next(&mut i, "--only")?)),
                "--diff" => {
                    let a = PathBuf::from(next(&mut i, "--diff")?);
                    let b = PathBuf::from(next(&mut i, "--diff")?);
                    cli.diff = Some((a, b));
                }
                "--sweep" => cli.sweep = Some(parse_sweep(&next(&mut i, "--sweep")?)?),
                other => return Err(format!("unknown argument `{other}`")),
            }
            i += 1;
        }
        // A typo that silently traced nothing would look like success.
        let unknown = cli
            .only
            .iter()
            .flatten()
            .chain(cli.trace.names())
            .find(|name| !scenarios::ALL.iter().any(|s| s.name == **name));
        match unknown {
            Some(name) => Err(format!("unknown scenario `{name}`")),
            None => Ok(cli),
        }
    }

    fn selected(&self) -> Vec<&'static harness::ScenarioSpec> {
        scenarios::ALL
            .iter()
            .filter(|s| {
                self.only
                    .as_ref()
                    .is_none_or(|only| only.iter().any(|n| n == s.name))
            })
            .collect()
    }

    fn run(&self) {
        if self.list {
            for s in scenarios::ALL {
                println!("{}", s.name);
            }
            return;
        }
        if let Some((a, b)) = &self.diff {
            match (harness::read_json(a), harness::read_json(b)) {
                (Ok(base), Ok(new)) => print!("{}", harness::render(&new, Some(&base))),
                (Err(e), _) | (_, Err(e)) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
            return;
        }
        if let Some(sweep) = &self.sweep {
            self.run_sweep(sweep);
            return;
        }

        let card = self.run_grid(BTreeMap::new());
        let baseline = root().join("quality_baseline.json");
        let base = harness::read_json(&baseline).ok();
        print!("{}", harness::render(&card, base.as_ref()));
        if base.is_none() {
            println!("\nno baseline at {}; `--bless` to write one", baseline.display());
        }

        if let Some(out) = &self.out {
            write(out, &card);
        }
        if self.bless {
            write(&baseline, &card);
        }
    }

    fn run_grid(&self, tuning: BTreeMap<String, f64>) -> Scorecard {
        let results: Vec<ScenarioResult> = self
            .selected()
            .iter()
            .map(|spec| {
                let ctx = Ctx {
                    cache_dir: self.out_dir.clone(),
                    out_dir: self.out_dir.clone(),
                    trace: self.trace.wants(spec.name),
                };
                eprintln!("running {}…", spec.name);
                ScenarioResult::new(spec.name, (spec.run)(&ctx))
            })
            .collect();
        Scorecard::new(results, tuning)
    }

    /// One column of scores per tunable value. Answers "which constant", not
    /// "did this commit help", and touches no committed file.
    fn run_sweep(&self, sweep: &Sweep) {
        let Some(original) = rts_lib::sim::get_tuning(&sweep.name) else {
            eprintln!("unknown tunable `{}`", sweep.name);
            std::process::exit(2);
        };
        let mut cards: Vec<(String, Scorecard)> = Vec::new();
        let mut value = sweep.lo;
        while value <= sweep.hi + sweep.step * 1e-3 {
            rts_lib::sim::set_tuning(&sweep.name, value);
            eprintln!("── {}={value} ──", sweep.name);
            let tuning = BTreeMap::from([(sweep.name.clone(), value as f64)]);
            cards.push((format!("{value:.4}"), self.run_grid(tuning)));
            value += sweep.step;
        }
        rts_lib::sim::set_tuning(&sweep.name, original);
        print!("{}", harness::render_sweep(&cards));
        println!("\n{} restored to {original}", sweep.name);
        if let Some(out) = &self.out {
            eprintln!("--out is ignored for sweeps ({} would be ambiguous)", out.display());
        }
    }
}

fn write(path: &PathBuf, card: &Scorecard) {
    match harness::write_json(path, card) {
        Ok(()) => println!("wrote {}", path.display()),
        Err(e) => {
            eprintln!("cannot write {}: {e}", path.display());
            std::process::exit(1);
        }
    }
}

/// Comma-separated scenario names, blanks dropped.
fn csv(arg: &str) -> Vec<String> {
    arg.split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect()
}

fn parse_sweep(arg: &str) -> Result<Sweep, String> {
    let bad = || format!("--sweep wants name=lo:hi:step, got `{arg}`");
    let (name, range) = arg.split_once('=').ok_or_else(bad)?;
    let parts: Vec<&str> = range.split(':').collect();
    let [lo, hi, step] = parts.as_slice() else {
        return Err(bad());
    };
    let num = |s: &str| s.parse::<f32>().map_err(|_| bad());
    let (lo, hi, step) = (num(lo)?, num(hi)?, num(step)?);
    if step <= 0.0 || hi < lo {
        return Err(format!("--sweep range `{range}` is empty"));
    }
    Ok(Sweep {
        name: name.to_string(),
        lo,
        hi,
        step,
    })
}
