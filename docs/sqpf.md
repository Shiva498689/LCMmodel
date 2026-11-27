# Sequential Pattern Formation (SqPF and SqPF′)

Registry key `SqPatternFormation`, shown as **Sequential Pattern Formation**. Code:
[`crates/lcm-algorithms/src/sq_pattern_formation.rs`](../crates/lcm-algorithms/src/sq_pattern_formation.rs).
Tests: [`crates/lcm-algorithms/tests/sq_pattern_formation.rs`](../crates/lcm-algorithms/tests/sq_pattern_formation.rs).

Source: P. Flocchini, A. Navarra, D. Pattanayak, F. Piselli, N. Santoro, *Universal Pattern Formation
by Oblivious Robots Under Sequential Schedulers*, arXiv:2412.10733v3 (20 Jul 2026). Page numbers
below are that PDF's.

The older `PatternFormation` entry, shown as *Pattern Formation (shared axes)*, is **not** this
algorithm. It is the original simulator's star formation, generalised to more shapes. Its robots share
the world's axes, find themselves by robot ID and count robots, three capabilities the paper's robots
do not have. It stops within 3 % of the swarm's scale rather than at an exact copy, and forms the shape
up to translation and scale only. It stays for the command line and the recorded runs in `fixtures/`;
the page does not offer it.

## 1. The robots (Section 2.1, p. 4)

| The paper | Here |
| --- | --- |
| "a private coordinate system of which it perceives itself at the origin" | The core's disoriented frames (`frames: "disoriented"`, which the registry requires): at **every Look** the robot gets a new random frame, with a random x-axis direction, unit of length (½ to 2 world units, log-uniform) and handedness, itself at the origin. The core hands `compute` the view and `eps` already in that frame and turns the destination back into world coordinates. |
| *Look*: "an instantaneous snapshot of the positions occupied by the robots expressed within a private coordinate system" | `snapshot()` gives the **set** Q of distinct occupied points in that frame, sorted by the robot's own coordinates. |
| *identical*: "do not have distinct identities that can be used during the computation" | `compute` reads only the positions in the view: never `look.me`, `view.id` or a count of robots. `destination(pattern, Q)` takes only points. Test `a_decision_uses_only_the_occupied_points_not_ids_or_counts` changes the robot's index, the ids, their order and the robots per point, and gets the same decision. |
| *disoriented*: "may not agree on a common coordinate system nor on chirality" | Frames differ in direction, unit and handedness, from robot to robot and from Look to Look. Shared axes (`global`), common chirality (`rotated`) and a compass are refused. Test `a_decision_does_not_depend_on_the_private_coordinate_system` checks that a destination is the same in every frame (Separate excepted, since by the paper it moves along the robot's own x-axis). Test `rotating_or_reflecting_the_start_rotates_or_reflects_the_result` checks whole runs: a start turned by 90° or 200°, or mirrored, ends in the turned or mirrored formation, point by point. |
| *oblivious*: "at the beginning of each cycle, a robot has no memory of past actions" | `destination` is a pure function; `SqPatternFormation` has no mutable state. |
| *silent* | No lights, no messages. |
| "robots without any type of multiplicity detection" | Robots on one point are one point of Q (test `robots_on_one_point_are_one_point_and_have_no_identity`). |
| "In this work we consider only non-rigid movements" (δ > 0, unknown to the robots) | The registry refuses `rigid_movement: true` or no `delta`. The core's adversary stops each move after at least δ. |
| Sequential scheduler 𝒮𝒬 (Section 2.2, p. 5): one robot per round, fair | The registry refuses every other scheduler: async, SSYNC, FSYNC and k-Async (UPF is unsolvable under FSYNC, hence under SSYNC and ASYNC: Theorem 1, p. 6). On the page, choosing the algorithm switches the scheduler to Sequential and greys it out. |
| A snapshot of all robots (Section 8.2, p. 24: "the robots can view the positions of all other robots") | Limited visibility is refused. |
| No faults in the model | Faulty robots are refused. |
| UPF: "form any arbitrary pattern P̂, given as their sole input and to no longer move", n ≥ \|P̂\| (Section 2.3, p. 5) | The pattern is `target_points`, or the corners of `target_pattern`. Any n ≥ k is allowed (CLI, config and page); n < k is refused. On the page, a new pattern starts with n = k and the Robots slider cannot go below k. Robots never terminate: the run stops when a whole epoch passes with no robot moving (`Stalled`), and the page then checks Q ∈ 𝓑(P̂) and says **Formed**. |

## 2. Paper line → code

| Paper | Code |
| --- | --- |
| Algorithm 1 l. 1–3 (and Algorithm 7 l. 3–4): \|Q\| < k → Separate | `destination` |
| Algorithm 1 l. 5: scale P̂ to ρ | `Pattern::place` (P̂ is kept with SEC radius 1) |
| Algorithm 1 l. 6–17 | `sqpf` |
| Procedure 2 Separate | `separate` |
| Procedure 3 Last | `last` |
| Procedure 4 Overlap | `overlap` |
| Procedure 5 Leader | `leader` |
| Procedure 6 Occupy, walker of Section 4.1.7, ranking of Section 4.1.6 | `occupy`, `Scene::ranking` |
| Section 3.3: S′, angular sequence, leader angular sequence, radiangular distance, radial detour | `Scene::rays`, `Scene::gaps`, `Scene::leader_seq`, `Scene::radiangular`, `Scene::robots_on_detour` |
| Section 3.3: μ̂ (α⁽ⁱ⁾, β⁽ⁱ⁾ pattern sequences) | `mu_hat` |
| Algorithm 7 SqPF′ (1 < \|P̂\| < 5) | `sqpf_prime` |

`Step` names the paper lines that decided each move (`Step::lines()`).

## 3. Where the paper leaves a choice, and what is done here

Each item is cited in the code.

1. **ψ(P̂) on one radius.** Section 3.3 does not order co-radial pattern points. Here they are
   listed from the centre outwards, in both turning directions. Then α⁽ⁱ⁾ of P̂ and β⁽ⁱ⁾ of its
   mirror image are equal, so robots of either handedness find the same μ̂. Reversing the list instead
   would let robots of opposite handedness disagree. A pattern point at O has no angle; it is left out
   of the sequence and ranked last.
2. **"Responsible for SEC(Q)"** (Procedures 5 and 6) means that removing the robot changes SEC(Q).
3. **Procedure Last, line 7.** The condition includes "q ∈ Op̂ₖ", so a literal reading returns `null`
   to every other robot. Such a robot then goes through Overlap and Leader and leaves its pattern point
   while the last robot is still on its way to p̂ₖ. In tests this cycled forever (pentagon, n = 5, non-rigid
   δ = 5). The text ("Once a robot located at Op̂ₖ is activated, it moves toward p̂ₖ"), Figure 1 (*Special
   case?* → Last → END) and the proof of Lemma 2 treat the special case as a property of the configuration.
   So in that case the robot on Op̂ₖ moves toward p̂ₖ and **every other robot stays** (`Step::FinalizeWait`).
4. **Procedure Last with more than k robots on SEC(Q)**: "not handled by this procedure"; Last returns `null`.
5. **Procedure Leader, line 16** fixes |Ou′| = ρ/2 and ∠u′Oq_j = ξ/3 but not the side of q_j.
   - With Γ: u′ goes on the side from which turning by ξ/3 in Γ's clockwise direction reaches q_j.
     Then the new λ(Q) induces the same clockwise direction and the same Γ, which is what the proof of
     Lemma 3 relies on ("This induces a leader angular sequence for Γ").
   - Without Γ: the robot at O turns counterclockwise in its own frame, and "any robot on SEC(Q)" (line 11)
     is the first in its own coordinates. Only that robot moves, so any choice is allowed.
   - The q_j of line 14 is λ(Q)'s, the one Overlap used. The two always coincide (a unique smallest
     angle of Γ between two robots is also Q's).
6. **Algorithm 7, equal distance deviations.** "r can consider one of them indiscriminately"
   (Section 5.1). A robot considers one in which it has a move, if there is one. Otherwise, from a
   symmetric start (a square start, a 4-point pattern), each robot could pick the overlap in which
   the *other* robot moves, and nobody would ever move. Seen in tests before this choice was made.
7. **Algorithm 7 has no "already formed" test.** A pattern with two or three pairs at the maximum
   distance (square, rectangle, equilateral triangle) has no unique maximum pair once formed, so
   line 7 would move it again. That contradicts "and to no longer move" (Section 2.3). So when Q ∈ 𝓑(P̂)
   the robot stays (`Step::PrimeFormed`). This is the only line added to the paper's pseudocode.
8. **Procedure Occupy.** "ρₗ contains other robots" counts free robots (not on a pattern point).
   Robots already on pattern points are settled, and counting them would leave line 13 with no free
   robot to pick. A path "contains other robots" if a robot other than the walker lies on it, the
   target point excepted. When every pattern point is occupied and |Q| > k, the walker joins p̂ₖ
   (Lemma 4: "or to position r on a pattern point already occupied if |Q| > |P̂|").
9. **Numbers.** Points are one point when they are closer than 10⁻⁹ of the snapshot's size
   (10⁻⁹·ρ in Compute), or closer than the simulator's minimum move, its `eps` (10⁻⁵ world units by
   default). Directions within 10⁻⁹ rad are one radius. The relative part does not depend on a robot's
   unit of length. The core gives each Look `eps` in that Look's unit, so it is the same world distance
   for every robot and decisions stay frame-independent.
   - Why the minimum move: the simulator drops any move shorter than `eps`. With the relative tolerance
     alone, a walker 3.6·10⁻⁶ from its pattern point was "not there yet", so it was told to move. The
     move was dropped, the walker never moved and the others waited for it forever. This happened in
     pentagon, n = 6, seed 7, δ = 2: stalled unformed after 23 epochs. A robot that cannot get closer to
     a point is on it.
   - The paper's robots move exactly, so this is a property of the simulation, not a change to the
     algorithm. The robots stop only when Q ∈ 𝓑(P̂) holds within that tolerance. The page's "formed?"
     test uses twice it (2·max(10⁻⁹·ρ, ε)), so it never calls "not formed" a configuration the robots
     stopped on as formed.
   - The test `a_walker_closer_than_the_minimum_move_is_on_its_pattern_point` replays that run.

10. **Procedure Last with more than k occupied positions on SEC(Q)** (choice 4) is a gap in the paper
    itself: "not handled by this procedure" (p. 9), and nothing else handles it. It should be raised
    with the authors.
11. **Procedure Leader: several robots heading for O.** Line 6–7 sends every robot that is not
    responsible for SEC(Q), has a free path to O and does not define θ₁ toward O. Under non-rigid
    moves a robot can be stopped on the way, so in later turns several robots can be on their way to
    O at once. The pseudocode does not say what happens then. Here it is followed literally: the
    first robot to reach O is the leader, and every other robot then sees O occupied and goes on
    with lines 9–19. Every run so far has formed the pattern, but the paper's proof does not cover
    this case. It should be raised with the authors.
12. **A new frame at every Look, and the end of a run.** A run ends when a whole epoch passes with no
    robot moving. Each decision is the same in every frame, except Separate (which always moves) and
    two tie-breaks: Leader line 11 "any robot on SEC(Q)" and Algorithm 7 line 24 "the closest robot
    position" when two robots are equally close. In those ties a robot picks the first in its own
    coordinates, which can change from Look to Look. In an exactly symmetric configuration a silent
    epoch could therefore end a run that a later Look would have continued. None of the runs below
    met this.

## 4. What the tests and runs show

- `cargo test -p lcm-algorithms --test sq_pattern_formation`: robot-model properties (frame
  invariance; ids, order and robots per point ignored), single decisions, whole runs of every shape
  (n = k and n = k + 2), stacked starts, a pattern with a point at the centre, rotated and mirrored
  starts (the formation turns with the start), Theorem 2's epoch bound (12 runs: pentagon, hexagon,
  star and cross, n = k, k + 1 and k + 3), and the settings that must be refused.
- During development, 1,056 further runs with the robots' own decision function, all forming the
  pattern:
  - Patterns: random 3-, 4-, 5-, 6-, 7- and 9-point patterns, pentagon + centre, co-radial and nested
    patterns, a rectangle.
  - Starts: random clouds, regular polygons, all robots on one circle, stacked points.
  - Moves: δ = 3 with random stops, and rigid.

  Another 243 runs on the final code covered the nine predefined shapes: n = k, k + 1 and k + 3;
  random starts, with and without stacked points; δ = 5 with random stops, and rigid. All 243 formed
  the pattern.
- 250 further runs after merging the core's scheduler and multiplicity work cover all nine shapes,
  n = 3 to 13, several seeds and δ = 2 to 5. With choice 9's minimum-move tolerance, all 250 formed the
  pattern. Run again with the core's disoriented frames (a new frame at every Look), all 250 formed the
  pattern, and the browser build and the native build agreed on the number of turns of every run.
  - Before the change, 249 formed and the pentagon run above stalled.
  - The change altered only that run's trace: the other 135 recorded traces are byte-identical.
- **Observation about Theorem 2's bound.** 2 of those 1,056 runs took longer than 2(n+1)⌈ρ/δ⌉+2 epochs:
  19 and 21 epochs against 14, both with rigid moves from a regular-pentagon start with n = k = 5.
  The pattern was still formed. The cause: Lemma 4 counts that "3 might be stuck to maintain the SEC and
  1 to maintain θ₁". When n = k = 5, one more robot is on p₁, so the only free walker is the robot that
  maintains θ₁. When it walks through O, θ₁ disappears and Leader runs again.

## 5. Running it

```sh
cargo test -p lcm-algorithms --test sq_pattern_formation
# one run from the command line (prints the final report as JSON)
cat > sqpf.json <<'EOF'
{"algorithm":"SqPatternFormation","target_pattern":"hexagon","num_of_robots":9,
 "scheduler":"sequential","frames":"disoriented","rigid_movement":false,"delta":5,"random_seed":7}
EOF
cargo run --release -p lcm-cli -- run sqpf.json
```

On the page: *Algorithm → Sequential Pattern Formation*, then pick a shape (or *Custom coordinates…*)
in the list next to it. The shape and the custom coordinates are settings (`target_pattern`,
`pattern_points`), so a copied or embedded link keeps them and *Defaults* resets them. The scheduler,
rigid movement, visibility, faults and the robots' axes (Disoriented) are fixed while it is chosen. A new
pattern starts with one robot per pattern point; the Robots slider can add more but not go below that.

## 6. Speed

The point tests a robot makes again and again ("is a robot within tolerance of this pattern point?")
use `PointIndex`, the points sorted by x as indices into the snapshot (no copy), so each test is a
binary search instead of a scan of all n points. Procedure Last stops at the second empty pattern
point (its special case allows one), the Look merges robots on one point with an ordered tree, and
the page's "formed?" test skips robot pairs whose distance cannot match the pattern's scale. Each of
these gives the same answer as the scan it replaces, so every decision is unchanged: recorded traces
of 13 runs (SqPF and SqPF′, n = k and n > k, stacked starts) are identical byte for byte.

At n = 500 a robot's Compute went from 15–21 ms to about 1 ms, and the page's per-frame "formed?" test
from about 6 s to under 1 ms. The number of turns is the algorithm's own (Theorem 2 bounds it by
2(n+1)⌈ρ/δ⌉ + 2 epochs of n turns; about 8.5·n epochs in practice), so large patterns still take long.
