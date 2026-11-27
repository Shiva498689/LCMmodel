//! Sequential Pattern Formation (SqPF / SqPF′, arXiv:2412.10733 Sections 2–5)
//! on the core's sequential scheduler.
//!
//! Three kinds of test: the robot model of Section 2.1 (private coordinate
//! systems, no identities, no multiplicity detection), single decisions of
//! the paper's procedures, and whole runs that must end with the pattern
//! formed and no robot moving. Last, the settings the registry must refuse.

use core::f64::consts::TAU;

use lcm_algorithms::sq_gathering::LocalFrame;
use lcm_algorithms::sq_pattern_formation::{
    destination, formed_placement, is_formed, pattern_of, sec, shape_points, snapshot, Pattern,
    SqPatternFormation, Step, KEY,
};
use lcm_core::geom::dist;
use lcm_core::{
    ActivationOrder, Algorithm, Budget, Frames, Look, Model, MultiplicityModel, PatternShape, Point,
    RobotState, Rng, SchedulerKind, Scratch, SimConfig, StopPolicy, StopReason, TurnGap, View,
};

fn p(x: f64, y: f64) -> Point {
    Point::new(x, y)
}

/// A run in the paper's model: sequential, disoriented robots (a new private
/// frame at every Look), non-rigid with δ, unlimited visibility, no faults.
fn config(shape: PatternShape, robots: u32, seed: u64, delta: f64) -> SimConfig {
    SimConfig {
        algorithm: KEY.to_owned(),
        target_pattern: shape,
        num_of_robots: robots,
        random_seed: seed,
        scheduler: SchedulerKind::Sequential,
        frames: Frames::Disoriented,
        activation_order: if seed.is_multiple_of(2) { ActivationOrder::Random } else { ActivationOrder::RoundRobin },
        turn_gap: TurnGap::None,
        rigid_movement: false,
        delta: Some(delta),
        stop_policy: StopPolicy::Random,
        max_turns: Some(400_000),
        ..SimConfig::default()
    }
}

/// The robot's decision, in world coordinates, seen through `frame`.
fn decide_world(pat: &Pattern, world: &[Point], me: usize, frame: &LocalFrame) -> (Point, Step) {
    let frame = LocalFrame { origin: world[me], ..*frame };
    let (d, step) = destination(pat, &snapshot(world, &frame, 0.0), 0.0);
    (frame.to_global(d), step)
}

/// A random private frame at `origin`, as the core draws one at every Look:
/// any direction, a unit between 1/2 and 2, either handedness.
fn random_frame(origin: Point, rng: &mut Rng) -> LocalFrame {
    LocalFrame {
        origin,
        angle: rng.uniform(0.0, TAU),
        scale: rng.uniform(-1.0, 1.0).exp2(),
        reflect: rng.random() < 0.5,
    }
}

fn random_cloud(n: usize, seed: u64) -> Vec<Point> {
    let mut rng = Rng::new(seed);
    (0..n).map(|_| p(rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0))).collect()
}

// ── the robot model (Section 2.1) ────────────────────────────────────────────

#[test]
fn a_decision_does_not_depend_on_the_private_coordinate_system() {
    // Disoriented robots: the same configuration seen with any direction of the
    // x-axis, any unit and either handedness gives the same destination. (Only
    // Separate, which by the paper moves "along its own x-axis at one unit
    // distance", is meant to depend on the frame; |Q| ≥ |P̂| here.)
    let patterns = [
        Pattern::new(&shape_points(PatternShape::Triangle)).unwrap(),
        Pattern::new(&shape_points(PatternShape::Star)).unwrap(),
        Pattern::new(&random_cloud(7, 99)).unwrap(),
    ];
    let frames = [
        LocalFrame { origin: p(0.0, 0.0), angle: 0.0, scale: 1.0, reflect: false },
        LocalFrame { origin: p(0.0, 0.0), angle: 1.1, scale: 0.3, reflect: false },
        LocalFrame { origin: p(0.0, 0.0), angle: 4.0, scale: 2.7, reflect: true },
        LocalFrame { origin: p(0.0, 0.0), angle: 2.5, scale: 1.0, reflect: true },
    ];
    let mut checked = 0;
    for (pi, pat) in patterns.iter().enumerate() {
        for seed in 0..12 {
            let world = random_cloud(pat.len() + (seed as usize % 3), 1000 + seed + 17 * pi as u64);
            for me in 0..world.len() {
                let (base, step) = decide_world(pat, &world, me, &frames[0]);
                for f in &frames[1..] {
                    let (d, s) = decide_world(pat, &world, me, f);
                    assert_eq!(s, step, "pattern {pi} seed {seed} robot {me}: step depends on the frame");
                    assert!(dist(d, base) < 1e-6, "pattern {pi} seed {seed} robot {me}: {d:?} vs {base:?}");
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 300);
}

#[test]
fn robots_on_one_point_are_one_point_and_have_no_identity() {
    // No multiplicity detection: two or five robots on a point look like one.
    let f = LocalFrame::global(p(3.0, 4.0));
    let two = [p(3.0, 4.0), p(0.0, 0.0), p(0.0, 0.0), p(9.0, 1.0)];
    let five = [p(0.0, 0.0), p(9.0, 1.0), p(0.0, 0.0), p(3.0, 4.0), p(0.0, 0.0), p(0.0, 0.0), p(0.0, 0.0)];
    assert_eq!(snapshot(&two, &f, 0.0), snapshot(&five, &f, 0.0));
    // The robot is the origin of its own snapshot.
    assert!(snapshot(&two, &f, 0.0).contains(&p(0.0, 0.0)));
}

#[test]
fn a_decision_uses_only_the_occupied_points_not_ids_or_counts() {
    // Identical robots without multiplicity detection (Section 2.1): the robot's
    // index, the ids in the view, their order and how many robots stand on a
    // point never change its decision.
    let pat = Pattern::new(&shape_points(PatternShape::Pentagon)).unwrap();
    let algo = SqPatternFormation { pattern: pat };
    let model = Model {
        precision: 5,
        eps: 1e-5,
        visibility: f64::INFINITY,
        width_bound: None,
        height_bound: None,
        multiplicity: MultiplicityModel::None,
    };
    let pts = random_cloud(6, 41);
    let decide = |order: &[usize], ids: &[u32], me: u32, stack: bool| {
        let mut view = View::default();
        let mut push = |id: u32, p: Point| {
            view.id.push(id);
            view.pos.push(p);
            view.state.push(RobotState::Wait);
            view.terminated.push(false);
            view.frozen.push(false);
        };
        for (&i, &id) in order.iter().zip(ids) {
            push(id, pts[i]);
            if stack {
                push(id + 100, pts[i]); // a second robot on the same point
            }
        }
        let me_index = view.pos.iter().position(|&q| q == pts[2]).unwrap();
        let look = Look { me, me_index, position: pts[2], view: &view, model: &model };
        algo.compute(&look, &mut Rng::new(1), &mut Scratch::default()).target
    };
    let base = decide(&[0, 1, 2, 3, 4, 5], &[0, 1, 2, 3, 4, 5], 2, false);
    assert_eq!(base, decide(&[5, 3, 1, 2, 0, 4], &[9, 7, 4, 8, 1, 3], 8, false), "ids and order");
    assert_eq!(base, decide(&[0, 1, 2, 3, 4, 5], &[0, 1, 2, 3, 4, 5], 77, true), "robots per point");
}

// ── single decisions ─────────────────────────────────────────────────────────

#[test]
fn separate_moves_robots_off_a_multiplicity_along_their_own_x_axis() {
    // Fewer occupied points than pattern points: Procedure Separate.
    let pat = Pattern::new(&shape_points(PatternShape::Pentagon)).unwrap();
    let q = [p(0.0, 0.0), p(5.0, 5.0), p(-7.0, 2.0)];
    let (d, step) = destination(&pat, &q, 0.0);
    assert_eq!(step, Step::Separate);
    assert_eq!(d, p(1.0, 0.0));
}

#[test]
fn a_formed_pattern_never_moves() {
    // Q ∈ 𝓑(P̂): rotated, reflected, scaled and moved copies of every shape.
    // Includes the square and the triangle, which have no unique maximum pair.
    for shape in [
        PatternShape::Line, PatternShape::Triangle, PatternShape::Square, PatternShape::Pentagon,
        PatternShape::Hexagon, PatternShape::Star, PatternShape::Circle, PatternShape::Cross,
        PatternShape::Heart,
    ] {
        let pts = shape_points(shape);
        let pat = Pattern::new(&pts).unwrap();
        for (angle, s, mirror) in [(0.3, 0.7, false), (2.0, 3.0, true)] {
            let world: Vec<Point> = pts
                .iter()
                .map(|q| {
                    let y = if mirror { -q.y } else { q.y };
                    p(40.0 + s * (q.x * f64::cos(angle) - y * f64::sin(angle)),
                      -9.0 + s * (q.x * f64::sin(angle) + y * f64::cos(angle)))
                })
                .collect();
            assert!(is_formed(&pat, &world, 0.0), "{shape:?}");
            let mut rng = Rng::new(3);
            for me in 0..world.len() {
                let frame = random_frame(world[me], &mut rng);
                let (d, step) = decide_world(&pat, &world, me, &frame);
                assert!(dist(d, world[me]) < 1e-9, "{shape:?}: robot {me} moves ({step:?})");
                assert!(matches!(step, Step::Formed | Step::PrimeFormed), "{shape:?}: {step:?}");
            }
        }
    }
}

#[test]
fn the_formed_shape_is_reported_in_the_patterns_own_order() {
    // What the page joins into the outline: the pattern's points where the robots stand, in
    // the order the pattern was given, i.e. a scaled, rotated or mirrored copy of P̂ point by
    // point (for a symmetric pattern the copy may start at another corner: the same outline).
    // Robots: P̂ reflected and scaled, one corner stacked (n > k).
    for pts in [shape_points(PatternShape::Pentagon), random_cloud(7, 5)] {
        let pat = Pattern::new(&pts).unwrap();
        let place = |q: &Point| p(7.0 + 2.0 * q.y, 3.0 + 2.0 * q.x); // swap x and y: a reflection
        let mut world: Vec<Point> = pts.iter().map(place).collect();
        world.push(world[2]);
        let got = formed_placement(&pat, &world, 0.0).expect("formed");
        assert_eq!(got.len(), pts.len());
        for g in &got {
            assert!(world.iter().any(|w| dist(*g, *w) < 1e-6), "every drawn point holds a robot");
        }
        let ratio = dist(got[0], got[1]) / dist(pts[0], pts[1]);
        for i in 0..pts.len() {
            for j in 0..pts.len() {
                let (a, b) = (dist(got[i], got[j]), ratio * dist(pts[i], pts[j]));
                assert!((a - b).abs() < 1e-6 * b.max(1.0), "points {i}, {j} not in the pattern's order");
            }
        }
        world[0].x += 5.0;
        assert!(formed_placement(&pat, &world, 0.0).is_none(), "not formed: nothing to draw");
    }
}

// ── whole runs ───────────────────────────────────────────────────────────────

fn run(cfg: &SimConfig) -> (StopReason, bool, u64) {
    let mut sim = lcm_algorithms::simulation(cfg).expect("a valid SqPF config");
    let stop = sim.advance(Budget { max_events: u64::MAX, until_time: f64::INFINITY });
    let pat = pattern_of(cfg).unwrap();
    (stop, is_formed(&pat, &sim.robots().pos, sim.model().eps), sim.epochs_completed())
}

#[test]
fn every_shape_is_formed_and_then_nothing_moves() {
    // The run ends because a whole epoch passed with no robot moving (robots
    // never terminate), and at that point the occupied points are the pattern.
    for shape in [
        PatternShape::Line, PatternShape::Triangle, PatternShape::Square, PatternShape::Pentagon,
        PatternShape::Hexagon, PatternShape::Star,
    ] {
        let k = u32::try_from(shape_points(shape).len()).unwrap();
        for (extra, seed, delta) in [(0, 1, 5.0), (2, 2, 20.0)] {
            let cfg = config(shape, k + extra, seed, delta);
            let (stop, formed, epochs) = run(&cfg);
            assert_eq!(stop, StopReason::Stalled, "{shape:?} n={} seed={seed}", k + extra);
            assert!(formed, "{shape:?} n={} seed={seed}: stopped after {epochs} epochs, not formed", k + extra);
        }
    }
}

#[test]
fn starting_multiplicities_are_separated_and_the_pattern_formed() {
    // Several robots per point at the start (C(0) may contain multiplicities).
    for (shape, seed) in [(PatternShape::Square, 3), (PatternShape::Hexagon, 4)] {
        let k = u32::try_from(shape_points(shape).len()).unwrap();
        let cfg = SimConfig { multiplicities: 2, multiplicity_size: 2, ..config(shape, k + 2, seed, 8.0) };
        let (stop, formed, _) = run(&cfg);
        assert_eq!(stop, StopReason::Stalled);
        assert!(formed, "{shape:?}");
    }
}

#[test]
fn a_custom_pattern_with_a_point_at_the_centre_is_formed() {
    let mut cfg = config(PatternShape::Star, 8, 5, 6.0);
    cfg.target_points = Some(vec![
        [0.0, 0.0], [10.0, 0.0], [3.0, 9.5], [-8.0, 6.0], [-8.0, -6.0], [3.0, -9.5], [5.0, 0.0],
    ]);
    let (stop, formed, _) = run(&cfg);
    assert_eq!(stop, StopReason::Stalled);
    assert!(formed);
}

#[test]
fn a_walker_closer_than_the_minimum_move_is_on_its_pattern_point() {
    // Regression: the walker ended 3.6·10⁻⁶ from its pattern point. That is above
    // 10⁻⁹·ρ, so it counted as "not there yet", but below the simulator's minimum
    // move (`eps` = 10⁻⁵), so its move was dropped: it never moved, the other
    // robots waited for it, and the run stalled unformed (turn 138, 23 epochs).
    let cfg = SimConfig {
        algorithm: KEY.to_owned(),
        target_pattern: PatternShape::Pentagon,
        num_of_robots: 6,
        random_seed: 7,
        scheduler: SchedulerKind::Sequential,
        frames: Frames::Disoriented,
        rigid_movement: false,
        delta: Some(2.0),
        ..SimConfig::default()
    };
    let (stop, formed, epochs) = run(&cfg);
    assert_eq!(stop, StopReason::Stalled);
    assert!(formed, "stopped after {epochs} epochs, not formed");
}

/// The robots' positions at the end of a run of `cfg`.
fn final_positions(cfg: &SimConfig) -> Vec<Point> {
    let mut sim = lcm_algorithms::simulation(cfg).expect("a valid SqPF config");
    let stop = sim.advance(Budget { max_events: u64::MAX, until_time: f64::INFINITY });
    assert_eq!(stop, StopReason::Stalled);
    sim.robots().pos.clone()
}

/// A rotation or reflection of the plane about the origin.
type Transform = fn(Point) -> Point;

fn rotate(q: Point, degrees: f64) -> Point {
    let (s, c) = degrees.to_radians().sin_cos();
    p(c * q.x - s * q.y, s * q.x + c * q.y)
}

#[test]
fn rotating_or_reflecting_the_start_rotates_or_reflects_the_result() {
    // Disoriented robots cannot tell a configuration from its rotation or mirror
    // image (Section 2.1), so the formed pattern turns with the start. A run that
    // read the world axes would form the same upright shape every time. The core
    // draws the same private frames in both runs and every decision is
    // frame-independent, so the final positions match point by point.
    let transforms: [(&str, Transform); 3] = [
        ("rotated 90°", |q| rotate(q, 90.0)),
        ("rotated 200°", |q| rotate(q, 200.0)),
        ("reflected", |q| p(q.x, -q.y)),
    ];
    for (shape, seed) in [(PatternShape::Pentagon, 11), (PatternShape::Hexagon, 12), (PatternShape::Star, 13)] {
        let k = shape_points(shape).len();
        let start = random_cloud(k, 500 + seed);
        let with_start = |pts: &[Point]| SimConfig {
            initial_positions: Some(pts.iter().map(|q| [q.x, q.y]).collect()),
            ..config(shape, u32::try_from(k).unwrap(), seed, 7.0)
        };
        let base = final_positions(&with_start(&start));
        let pat = pattern_of(&with_start(&start)).unwrap();
        assert!(is_formed(&pat, &base, 1e-5), "{shape:?}: not formed");
        let (_, rho) = sec(&base);
        for (name, f) in transforms {
            let moved: Vec<Point> = start.iter().map(|&q| f(q)).collect();
            let got = final_positions(&with_start(&moved));
            assert!(is_formed(&pat, &got, 1e-5), "{shape:?} {name}: not formed");
            for (b, g) in base.iter().zip(&got) {
                assert!(dist(f(*b), *g) < 1e-6 * rho, "{shape:?} {name}: {:?} vs {g:?}", f(*b));
            }
        }
    }
}

#[test]
fn every_run_stays_within_theorem_2s_bound() {
    // Theorem 2 (p. 17): for |P| >= 5, SqPF forms the pattern in at most
    // 2(n+1)⌈ρ/δ⌉ + 2 epochs. ρ is the SEC radius of the start. The simulator notices
    // the end only after one whole epoch with no move, so that epoch is added.
    for shape in [PatternShape::Pentagon, PatternShape::Hexagon, PatternShape::Star, PatternShape::Cross] {
        let k = u32::try_from(shape_points(shape).len()).unwrap();
        for (extra, seed, delta) in [(0, 21, 40.0), (1, 22, 25.0), (3, 23, 60.0)] {
            let n = k + extra;
            let cfg = config(shape, n, seed, delta);
            let mut sim = lcm_algorithms::simulation(&cfg).unwrap();
            let (_, rho) = sec(&sim.robots().pos);
            let stop = sim.advance(Budget { max_events: u64::MAX, until_time: f64::INFINITY });
            let pat = pattern_of(&cfg).unwrap();
            assert_eq!(stop, StopReason::Stalled);
            assert!(is_formed(&pat, &sim.robots().pos, sim.model().eps), "{shape:?} n={n}: not formed");
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let bound = 2 * (u64::from(n) + 1) * (rho / delta).ceil() as u64 + 2;
            let epochs = sim.epochs_completed();
            assert!(epochs <= bound + 1, "{shape:?} n={n} δ={delta}: {epochs} epochs, bound {bound}");
        }
    }
}

// ── settings the paper's model does not have ─────────────────────────────────

#[test]
fn the_registry_refuses_settings_outside_the_papers_model() {
    let ok = config(PatternShape::Pentagon, 5, 1, 5.0);
    assert!(lcm_algorithms::simulation(&ok).is_ok());
    let refused = [
        ("async", SimConfig { scheduler: SchedulerKind::Async, ..ok.clone() }),
        ("shared axes", SimConfig { frames: Frames::Global, ..ok.clone() }),
        ("common chirality", SimConfig { frames: Frames::Rotated, ..ok.clone() }),
        ("compass", SimConfig { frames: Frames::Compass, ..ok.clone() }),
        ("rigid", SimConfig { rigid_movement: true, ..ok.clone() }),
        ("no delta", SimConfig { delta: None, ..ok.clone() }),
        ("limited visibility", SimConfig { visibility_radius: Some(50.0), ..ok.clone() }),
        ("faults", SimConfig { num_of_faults: 1, ..ok.clone() }),
        ("fewer robots than points", SimConfig { num_of_robots: 4, ..ok.clone() }),
        ("one point", SimConfig { target_points: Some(vec![[1.0, 1.0]]), ..ok.clone() }),
        ("repeated point", SimConfig { target_points: Some(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]]), ..ok.clone() }),
    ];
    for (why, cfg) in refused {
        assert!(lcm_algorithms::simulation(&cfg).is_err(), "{why} must be refused");
    }
    let message = lcm_algorithms::simulation(&SimConfig { scheduler: SchedulerKind::Async, ..ok })
        .err()
        .unwrap()
        .to_string();
    assert!(message.contains("sequential scheduler"), "{message}");
}

#[test]
fn the_page_is_told_to_fix_the_scheduler_and_non_rigid_moves() {
    assert_eq!(lcm_algorithms::requirements(KEY), (true, true));
    assert!(lcm_algorithms::needs_non_rigid(KEY));
    assert!(lcm_algorithms::needs_disoriented(KEY));
    assert!(!lcm_algorithms::needs_disoriented("SqGathering"));
    assert!(!lcm_algorithms::needs_non_rigid("SqGathering"));
}
