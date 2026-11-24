//! Private frames (docs/core-schema.md §2, "frames"): what a robot's Look
//! contains in each mode, and the sequential scheduler with disoriented
//! robots, rigid and non-rigid (the setting of SqPF, arXiv:2412.10733).

use std::cell::RefCell;
use std::rc::Rc;

use lcm_core::geom::dist;
use lcm_core::Rng;
use lcm_core::{
    Algorithm, Budget, Decision, Frames, Look, OverlayUpdate, Plan, Point, SchedulerKind, Scratch,
    SimConfig, Simulation, StopPolicy, StopReason,
};

/// What one Look contained.
#[derive(Clone, Debug)]
struct Seen {
    me: u32,
    me_index: usize,
    position: Point,
    view: Vec<Point>,
    eps: f64,
    visibility: f64,
    width_bound: Option<f64>,
    height_bound: Option<f64>,
}

/// Records every Look and stays where it is, so the global positions never
/// change and two Looks of one robot see the same configuration.
struct Recorder(Rc<RefCell<Vec<Seen>>>);

impl Algorithm for Recorder {
    fn key(&self) -> &'static str {
        "Recorder"
    }

    fn compute(&self, look: &Look<'_>, _: &mut Rng, _: &mut Scratch) -> Decision {
        self.0.borrow_mut().push(Seen {
            me: look.me,
            me_index: look.me_index,
            position: look.position,
            view: look.view.pos.clone(),
            eps: look.model.eps,
            visibility: look.model.visibility,
            width_bound: look.model.width_bound,
            height_bound: look.model.height_bound,
        });
        Decision {
            target: look.position,
            terminate: false,
            overlay: OverlayUpdate::Keep,
        }
    }
}

const START: [[f64; 2]; 4] = [[120.0, 80.0], [-40.0, 200.0], [300.0, -150.0], [10.0, 10.0]];

fn recorded(frames: Frames, scheduler: SchedulerKind) -> Vec<Seen> {
    let config = SimConfig {
        num_of_robots: 4,
        initial_positions: Some(START.to_vec()),
        visibility_radius: Some(1000.0),
        frames,
        scheduler,
        compass_error: 30.0,
        compass_static: false,
        ..SimConfig::default()
    };
    let looks = Rc::new(RefCell::new(Vec::new()));
    let plan = Plan::single(Box::new(Recorder(Rc::clone(&looks))));
    let mut sim = Simulation::new(&config, plan).expect("valid configuration");
    let stop = sim.advance(Budget {
        max_events: 400,
        until_time: f64::INFINITY,
    });
    let looks = looks.borrow().clone();
    // Sequential: one epoch, then the stall check ends the run (nobody moves).
    // Async has no stall check: every robot looks many times.
    assert!(
        looks.len() >= 4,
        "every robot looked: {} looks, {stop:?}",
        looks.len()
    );
    looks
}

fn global(i: u32) -> Point {
    let p = START[i as usize];
    Point::new(p[0], p[1])
}

fn same(a: Point, b: Point) -> bool {
    dist(a, b) < 1e-9
}

#[test]
fn a_global_look_is_the_original_one() {
    for seen in recorded(Frames::Global, SchedulerKind::Sequential) {
        assert!(same(seen.position, global(seen.me)));
        assert!(seen.width_bound.is_some() && seen.height_bound.is_some());
        for (k, p) in seen.view.iter().enumerate() {
            assert!(same(*p, global(u32::try_from(k).unwrap())));
        }
    }
}

/// With a private frame a robot learns nothing global: it is at the origin,
/// the world box is hidden, the others are in its own axes and unit (eps and
/// visibility scaled to match), and every Look draws a new frame.
#[test]
fn a_private_frame_leaks_no_global_information() {
    for frames in [Frames::Rotated, Frames::Disoriented, Frames::Compass] {
        for scheduler in [SchedulerKind::Sequential, SchedulerKind::Async] {
            let looks = recorded(frames, scheduler);
            for seen in &looks {
                let tag = format!("{frames:?} {scheduler:?} robot {}", seen.me);
                assert_eq!(seen.position, Point::new(0.0, 0.0), "{tag}");
                assert_eq!(seen.view[seen.me_index], Point::new(0.0, 0.0), "{tag}");
                assert_eq!((seen.width_bound, seen.height_bound), (None, None), "{tag}");
                // Every robot here is away from the global origin: its view is
                // not the global configuration.
                for (k, p) in seen.view.iter().enumerate() {
                    assert!(!same(*p, global(u32::try_from(k).unwrap())), "{tag}");
                }
                // Distances, eps and visibility share one unit.
                let (a, b) = (seen.view[0], seen.view[1]);
                let scale = dist(a, b) / dist(global(0), global(1));
                assert!((seen.eps / 1e-5 - scale).abs() < 1e-9 * scale, "{tag}");
                assert!(
                    (seen.visibility / 1000.0 - scale).abs() < 1e-9 * scale,
                    "{tag}"
                );
            }
            // Robots never move, so two Looks of one robot differ only by
            // their frames: a robot's view of the others must change.
            for me in 0..4 {
                let mine: Vec<&Seen> = looks.iter().filter(|s| s.me == me).collect();
                if scheduler == SchedulerKind::Async {
                    assert!(mine.len() >= 2, "robot {me} looked again");
                }
                let other = usize::from(me == 0);
                assert!(
                    mine.windows(2)
                        .all(|w| !same(w[0].view[other], w[1].view[other])),
                    "{frames:?} {scheduler:?}: robot {me} kept its frame between Looks"
                );
            }
        }
    }
}

fn gathering(positions: &[[f64; 2]], tweak: impl Fn(&mut SimConfig)) -> SimConfig {
    let mut config = SimConfig {
        algorithm: "Gathering".to_owned(),
        num_of_robots: u32::try_from(positions.len()).unwrap(),
        initial_positions: Some(positions.to_vec()),
        scheduler: SchedulerKind::Sequential,
        ..SimConfig::default()
    };
    tweak(&mut config);
    config
}

fn run(config: &SimConfig) -> (Simulation, StopReason) {
    let mut sim = lcm_algorithms::simulation(config).expect("accepted");
    let stop = sim.advance(Budget {
        max_events: 200_000,
        until_time: f64::INFINITY,
    });
    (sim, stop)
}

/// The sequential scheduler with disoriented robots, rigid and non-rigid
/// (`delta`, every stop policy): accepted, and since Gathering only uses
/// distances, the run is the global one up to rounding. The frames draw from
/// their own generator, so the scheduler's draws are unchanged.
#[test]
fn sequential_disoriented_runs_rigid_and_non_rigid() {
    let positions = [
        [0.0, 0.0],
        [90.0, 10.0],
        [30.0, 70.0],
        [-50.0, 40.0],
        [20.0, -60.0],
    ];
    let movements: [&dyn Fn(&mut SimConfig); 4] = [
        &|_| {},
        &|c| {
            c.rigid_movement = false;
            c.delta = Some(5.0);
        },
        &|c| {
            c.rigid_movement = false;
            c.delta = Some(5.0);
            c.stop_policy = StopPolicy::Delta;
        },
        &|c| {
            c.rigid_movement = false;
            c.delta = Some(5.0);
            c.stop_policy = StopPolicy::Fraction;
            c.stop_fraction = 0.3;
        },
    ];
    for (m, movement) in movements.iter().enumerate() {
        let plain = gathering(&positions, |c| movement(c));
        let (a, stop_a) = run(&plain);
        let disoriented = gathering(&positions, |c| {
            movement(c);
            c.frames = Frames::Disoriented;
        });
        disoriented.validate().expect("accepted");
        let (b, stop_b) = run(&disoriented);
        assert_eq!(stop_a, stop_b, "movement {m}");
        assert_eq!(a.turn_count(), b.turn_count(), "movement {m}");
        for (p, q) in a.robots().pos.iter().zip(&b.robots().pos) {
            assert!(dist(*p, *q) < 1e-6, "movement {m}: {p:?} vs {q:?}");
        }
    }
}

/// Leaving `frames` out is the same run as `frames: "global"`, event for event.
#[test]
fn default_frames_are_the_global_frame() {
    let positions = [[0.0, 0.0], [90.0, 10.0], [30.0, 70.0]];
    let unset = gathering(&positions, |c| {
        c.rigid_movement = false;
        c.delta = Some(5.0);
    });
    let explicit = SimConfig {
        frames: Frames::Global,
        ..unset.clone()
    };
    assert_eq!(unset, explicit);
    let (mut a, mut b) = (
        lcm_algorithms::simulation(&unset).unwrap(),
        lcm_algorithms::simulation(&explicit).unwrap(),
    );
    for _ in 0..2000 {
        let (x, y) = (a.step(), b.step());
        assert_eq!((x.robot, x.kind, x.outcome), (y.robot, y.kind, y.outcome));
    }
    assert_eq!(a.robots().pos, b.robots().pos);
}
