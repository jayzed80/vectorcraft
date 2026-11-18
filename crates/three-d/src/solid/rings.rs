use std::collections::BinaryHeap;

use super::{FillRule, MAX_SOLID_FACES, ParamCurve, PathData, Point, Rect, Vec2};

/// Outline points Extrude & Bevel and Inflate sample at most, and contours they take at most.
pub(super) const MAX_SOLID_POINTS: usize = 65536;

/// Outline points Rotate samples at most: it only turns the outline, so an outlined paragraph fits.
pub(super) const MAX_PLANE_POINTS: usize = 65536;

/// Pieces a twisted wall's strip is cut into along one edge at most.
pub(super) const MAX_PIECES: usize = 64;

/// Pieces a curve takes at most, where its tolerance asks for them.
pub(super) const CURVE_PIECES: u32 = 32;

/// The fewest pieces a curve of a solid over its budget is held to: a curve whose tolerance asks
/// for fewer takes fewer.
pub(super) const MIN_CURVE_PIECES: u32 = 4;

/// How far an inset corner's miter reaches at most, as a multiple of the inset. [`square_corners`]
/// cuts a corner whose miter would reach further square across instead.
const MITER_LIMIT: f64 = 4.0;

const TOO_COMPLEX_SHAPE: &str = "The shape is too complex for a 3D effect";
/// The error a solid past its limits returns, and a 3D effect whose solids together pass
/// [`crate::MAX_OBJECT_FACES`].
pub const TOO_COMPLEX_SURFACE: &str = "The 3D surface is too complex; simplify the shape";

/// How many straight pieces follow `c` within `tolerance` (Wang's bound on a cubic's distance from
/// its chords), 1 to [`CURVE_PIECES`].
fn pieces(c: vectorcraft_geom::CubicBez, tolerance: f64) -> u32 {
    let d = |a: Point, b: Point, c: Point| (a.to_vec2() - 2.0 * b.to_vec2() + c.to_vec2()).hypot();
    let bend = d(c.p0, c.p1, c.p2).max(d(c.p1, c.p2, c.p3));
    let n = (0.75 * bend / tolerance).sqrt().ceil();
    if n.is_finite() { (n as u32).clamp(1, CURVE_PIECES) } else { CURVE_PIECES }
}

/// The most pieces each curve of a solid's outline takes, counted on `path` before any point is
/// sampled: [`CURVE_PIECES`] when the outline fits within [`MAX_SOLID_POINTS`] points and
/// [`MAX_SOLID_FACES`] faces, else the largest number down to [`MIN_CURVE_PIECES`] that fits, the
/// same for every curve. A curve whose tolerance (see [`sample`]) asks for fewer pieces takes
/// fewer. The faces are counted as `per_edge` for each edge of the outline and `fixed` more.
///
/// An error when the outline takes more than [`MAX_SOLID_POINTS`] points, or contours, with
/// every curve held to [`MIN_CURVE_PIECES`], and when its faces do not fit with that many.
pub(super) fn curve_pieces(path: &PathData, size: f64, per_edge: usize, fixed: usize) -> Result<u32, String> {
    let tolerance = (size / 2000.0).max(0.05);
    // How many curves ask for each number of pieces.
    let mut curves = [0usize; CURVE_PIECES as usize + 1];
    let (mut lines, mut segments, mut open) = (0usize, 0usize, 0usize);
    for (contours, sp) in path.subpaths.iter().enumerate() {
        if contours >= MAX_SOLID_POINTS {
            return Err(TOO_COMPLEX_SHAPE.into());
        }
        // A subpath of one anchor draws nothing, as in [`sample`].
        if sp.anchors.len() < 2 {
            continue;
        }
        if sp.anchors.iter().any(|a| [a.p, a.h_in, a.h_out].iter().any(|q| !q.x.is_finite() || !q.y.is_finite())) {
            return Err("The shape contains invalid coordinates".into());
        }
        let count = if sp.closed { sp.anchors.len() } else { sp.anchors.len() - 1 };
        // Every segment takes a point at least.
        segments = segments.saturating_add(count);
        if segments > MAX_SOLID_POINTS {
            return Err(TOO_COMPLEX_SHAPE.into());
        }
        open += usize::from(!sp.closed);
        for i in 0..count {
            if sp.segment_is_line(i) {
                lines += 1;
            } else if let Some(n) = curves.get_mut(pieces(sp.segment(i), tolerance) as usize) {
                *n += 1;
            }
        }
    }
    let points = |most: u32| -> usize {
        curves.iter().enumerate().fold(lines, |sum, (asked, n)| sum.saturating_add(n.saturating_mul(asked.min(most as usize))))
    };
    if points(MIN_CURVE_PIECES) > MAX_SOLID_POINTS {
        return Err(TOO_COMPLEX_SHAPE.into());
    }
    // An open contour is closed by one more edge.
    let faces = |most: u32| points(most).saturating_add(open).saturating_mul(per_edge).saturating_add(fixed);
    (MIN_CURVE_PIECES..=CURVE_PIECES)
        .rev()
        .find(|most| points(*most) <= MAX_SOLID_POINTS && faces(*most) <= MAX_SOLID_FACES)
        .ok_or_else(|| TOO_COMPLEX_SURFACE.into())
}

/// Contours thinner than this on average (twice their area over their perimeter) are dropped when
/// overlaps are resolved: the debris a boolean leaves where two curves coincide, far below any
/// art. Pathfinder's default precision would drop real hairlines too.
pub(super) const DEBRIS: f64 = 1e-6;

/// The rings of a solid's profile, as [`rings`], with contours that meet merged first. Type often
/// draws one glyph as overlapping contours (a "k"'s arm across its stem), and [`rings`] tells a
/// contour's side from one point of it, so a contour that meets another, or itself, must be cut
/// where they meet. Most art has no such contours, and the boolean costs more than the rest of the
/// solid, so it runs only when [`edges_meet`] finds some. Its answer already runs holes against
/// their outlines, so it is only turned as a whole ([`wound`]), never judged a ring at a time.
///
/// The `limit` is checked on the profile as drawn, before the boolean, and again on the merged
/// outline, which can have more points (bars crossing in a grid trace every hole between them):
/// past it the shape is too complex, since its contours as drawn cross and can't be built as they
/// are. A profile the boolean refuses is built from its contours as drawn, as it was before
/// contours were merged. An empty answer is an answer: contours that cancel (a square drawn twice
/// under even-odd), or that are all thinner than [`DEBRIS`], fill nothing, so nothing is built.
/// Each curve takes at most `most` pieces, on the profile as drawn and on the merged outline.
pub(super) fn solid_rings(path: &PathData, rule: FillRule, size: f64, limit: usize, most: u32) -> Result<Vec<Vec<Point>>, String> {
    Ok(solid_rings_traced(path, rule, size, limit, most)?.into_iter().map(|r| r.points).collect())
}

/// Do the `sampled` rings of `closed` (the profile as the boolean takes it) meet, so that it must
/// go through the boolean? Within twice the boolean's tolerance ([`touch`]): pathops first snaps
/// anchors onto nearby heights (by less than the tolerance), then the sweep joins edges within it
/// along the sweep line, so straight edges a little more than it apart can come out as one: at
/// most sin θ + cos θ ≤ √2 times it, θ the edges' angle to the sweep line (about 1.4 at 45°,
/// measured; the same when pathops turns the sweep a quarter turn, which it does for some art of
/// 2,048 segments or more). Straight contours it might join are merged; those it leaves apart cost
/// a boolean that builds the same shape. Curves are judged on their sampled outline (see
/// [`sample`]), which can lie a little inside them (up to the sampling tolerance, or more for a
/// long curve held to 32 pieces), so curves that touch or overlap by less than that can be judged
/// apart and built as drawn.
pub(super) fn meets(sampled: &[Vec<Point>], closed: &PathData) -> bool {
    edges_meet(sampled, 2.0 * touch(closed))
}

/// `path` as the boolean should take it: each open subpath closed by a straight line, as [`sample`]
/// and the renderer's fill close it (the boolean would close it along the end anchors' handles).
/// Borrowed when nothing needs changing: every subpath closed, a single anchor (which the boolean
/// skips), or open with no handle on the closing side of either end anchor (which the boolean
/// already closes straight).
pub(super) fn closed_straight(path: &PathData) -> std::borrow::Cow<'_, PathData> {
    // The closing segment (last anchor to first) as the boolean will draw it once closed; the
    // length test comes first, so a subpath of fewer than two anchors never reaches the index.
    let straight = |sp: &vectorcraft_geom::SubPath| sp.closed || sp.anchors.len() < 2 || sp.segment_is_line(sp.anchors.len() - 1);
    if path.subpaths.iter().all(straight) {
        return std::borrow::Cow::Borrowed(path);
    }
    let mut path = path.clone();
    for sp in path.subpaths.iter_mut().filter(|sp| !sp.closed) {
        if let Some(last) = sp.anchors.last_mut() {
            last.h_out = last.p;
        }
        if let Some(first) = sp.anchors.first_mut() {
            first.h_in = first.p;
        }
        sp.closed = true;
    }
    std::borrow::Cow::Owned(path)
}

/// The boolean's rings turned so the filled side is on each one's left. Its holes already run
/// against their outlines, so only the direction of the whole is chosen: the one whose rings
/// enclose positive area in all.
pub(super) fn wound<R: Turnable>(mut rings: Vec<R>) -> Vec<R> {
    rings.retain(|r| signed_area(r.points()).abs() > 1e-9);
    if rings.iter().map(|r| signed_area(r.points())).sum::<f64>() < 0.0 {
        rings.iter_mut().for_each(Turnable::turn_around);
    }
    rings
}

/// The boolean's sweep tolerance for `path` as it is given to it (closed by [`closed_straight`]):
/// 64 times the rounding of the largest coordinate it reads, every anchor and handle of each
/// subpath of two anchors or more, and at least 1e-6 (pathops' `eps_for`). For a path [`sample`]
/// accepts (every point finite).
pub(super) fn touch(path: &PathData) -> f64 {
    let m = path
        .subpaths
        .iter()
        .filter(|sp| sp.anchors.len() >= 2)
        .flat_map(|sp| &sp.anchors)
        .flat_map(|a| [a.p, a.h_in, a.h_out])
        .fold(0.0f64, |m, q| m.max(q.x.abs()).max(q.y.abs()));
    (m * f64::EPSILON * 64.0).max(1e-6)
}

/// Most pairs of edges [`edges_meet`] compares. Past it the answer is yes, and the boolean, with
/// its own budget, decides.
const MAX_EDGE_PAIRS: usize = 1 << 22;

/// Do any two edges of `rings` meet, within `touch` (at that distance or closer): a contour crossing, touching or running
/// along another, or itself? Neighbouring edges of a ring share a corner and meet only where one
/// folds back along the other. Edges are swept in order of their left ends, so only pairs whose
/// bounds overlap across x are compared.
pub(super) fn edges_meet(rings: &[Vec<Point>], touch: f64) -> bool {
    struct Edge {
        ring: usize,
        index: usize,
        a: Point,
        b: Point,
        x0: f64,
        x1: f64,
    }
    let mut edges: Vec<Edge> = rings
        .iter()
        .enumerate()
        .flat_map(|(ring, r)| {
            r.iter().zip(r.iter().cycle().skip(1)).enumerate().map(move |(index, (a, b))| Edge {
                ring,
                index,
                a: *a,
                b: *b,
                x0: a.x.min(b.x),
                x1: a.x.max(b.x),
            })
        })
        .collect();
    edges.sort_by(|e, f| e.x0.total_cmp(&f.x0));
    let len = |ring: usize| rings.get(ring).map_or(0, Vec::len);
    let mut pairs = 0usize;
    for (i, e) in edges.iter().enumerate() {
        for f in edges.iter().skip(i + 1) {
            if f.x0 > e.x1 + touch {
                break;
            }
            pairs += 1;
            if pairs > MAX_EDGE_PAIRS {
                return true;
            }
            if e.a.y.min(e.b.y) > f.a.y.max(f.b.y) + touch || f.a.y.min(f.b.y) > e.a.y.max(e.b.y) + touch {
                continue;
            }
            let n = len(e.ring);
            let neighbours = e.ring == f.ring && ((e.index + 1) % n == f.index || (f.index + 1) % n == e.index);
            if neighbours {
                // They share a corner; they meet only if one runs back along the other.
                let (shared, p, q) = if e.b == f.a { (e.b, e.a, f.b) } else { (e.a, e.b, f.a) };
                if super::build::segment_distance(q, shared, p) <= touch || super::build::segment_distance(p, shared, q) <= touch {
                    return true;
                }
            } else if segments_meet(e.a, e.b, f.a, f.b, touch) {
                return true;
            }
        }
    }
    false
}

/// Do segments `a`–`b` and `c`–`d` cross, or come within `touch` of each other?
fn segments_meet(a: Point, b: Point, c: Point, d: Point, touch: f64) -> bool {
    let side = |p: Point, q: Point, r: Point| (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    let (d1, d2, d3, d4) = (side(c, d, a), side(c, d, b), side(a, b, c), side(a, b, d));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return true;
    }
    use super::build::segment_distance as dist;
    dist(a, c, d).min(dist(b, c, d)).min(dist(c, a, b)).min(dist(d, a, b)) <= touch
}

/// Where one edge of a sampled ring lies on the curve it follows: the piece of `curve` from `t0`
/// to `t1` (`t1` below `t0` on a ring that runs against its curve).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Span {
    pub(super) curve: vectorcraft_geom::CubicBez,
    pub(super) t0: f64,
    pub(super) t1: f64,
}

/// A sampled ring with, for each edge (point `i` to point `i + 1`, the last back to the first),
/// the piece of curve it follows, or `None` for a straight edge.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Ring {
    pub(super) points: Vec<Point>,
    pub(super) spans: Vec<Option<Span>>,
}

/// A ring that [`orient`] and [`wound`] can turn around.
pub(super) trait Turnable {
    fn points(&self) -> &[Point];
    fn turn_around(&mut self);
}

impl Turnable for Vec<Point> {
    fn points(&self) -> &[Point] {
        self
    }
    fn turn_around(&mut self) {
        self.reverse();
    }
}

impl Turnable for Ring {
    fn points(&self) -> &[Point] {
        &self.points
    }
    fn turn_around(&mut self) {
        // The ring keeps its first point, an anchor of the art, so that a run of edges along one
        // curve doesn't start part way along it. Edge `j` of the turned ring is edge `n - 1 - j`
        // of the ring, run the other way.
        self.points.reverse();
        if !self.points.is_empty() {
            self.points.rotate_right(1);
        }
        self.spans.reverse();
        for s in self.spans.iter_mut().flatten() {
            std::mem::swap(&mut s.t0, &mut s.t1);
        }
    }
}

/// [`solid_rings`] with the piece of curve each edge follows.
pub(super) fn solid_rings_traced(path: &PathData, rule: FillRule, size: f64, limit: usize, most: u32) -> Result<Vec<Ring>, String> {
    let sampled = sample_traced(path, size, limit, most)?;
    let closed = closed_straight(path);
    let points: Vec<Vec<Point>> = sampled.iter().map(|r| r.points.clone()).collect();
    if !meets(&points, &closed) {
        return orient(sampled, rule);
    }
    match vectorcraft_pathops::try_unite_all_with(&[(&closed, rule)], DEBRIS) {
        Ok(merged) => sample_traced(&merged, size, limit, most).map(wound),
        Err(_) => orient(sampled, rule),
    }
}

/// The rings of a shape: closed polygons with the filled side on their left, (-dy, dx): outer
/// outlines run one way and holes the other, so nonzero winding fills them as the art is. Curves
/// are followed within a tolerance of 1/2000 of `size` (at least 0.05 pt), so small curves take
/// few points and large ones more; more than `limit` points in all is refused.
///
/// Under the nonzero `rule` an outline with filled art on both sides is no edge of the shape and
/// is left out: a contour inside another that runs the same way doesn't cut a hole, as it would
/// if nesting alone decided (#1724).
pub(super) fn rings(path: &PathData, rule: FillRule, size: f64, limit: usize) -> Result<Vec<Vec<Point>>, String> {
    orient(sample(path, size, limit, CURVE_PIECES)?, rule)
}

/// The closed polygons that follow `path`'s subpaths, as [`rings`] samples them, each running as
/// its subpath does, each curve in at most `most` pieces.
pub(super) fn sample(path: &PathData, size: f64, limit: usize, most: u32) -> Result<Vec<Vec<Point>>, String> {
    Ok(sample_traced(path, size, limit, most)?.into_iter().map(|r| r.points).collect())
}

/// [`sample`] with the piece of curve each edge follows.
fn sample_traced(path: &PathData, size: f64, limit: usize, most: u32) -> Result<Vec<Ring>, String> {
    let tolerance = (size / 2000.0).max(0.05);
    let mut count = 0;
    let mut out = vec![];
    // A subpath of one anchor draws nothing, closed or not (the renderer and the boolean skip it).
    for sp in path.subpaths.iter().filter(|sp| sp.anchors.len() >= 2) {
        let segments = if sp.closed { sp.anchors.len() } else { sp.anchors.len() - 1 };
        if segments > limit {
            return Err(TOO_COMPLEX_SHAPE.into());
        }
        // Each point with the piece of curve of the edge that ends at it.
        let mut ring: Vec<(Point, Option<Span>)> = vec![];
        if let Some(first) = sp.anchors.first() {
            ring.push((first.p, None));
        }
        for i in 0..segments {
            let curve = sp.segment(i);
            let line = sp.segment_is_line(i);
            let samples = if line { 1 } else { pieces(curve, tolerance).min(most.max(1)) };
            count += samples as usize;
            if count > limit {
                return Err(TOO_COMPLEX_SHAPE.into());
            }
            for k in 1..=samples {
                let (t0, t1) = (f64::from(k - 1) / f64::from(samples), f64::from(k) / f64::from(samples));
                ring.push((curve.eval(t1), (!line).then_some(Span { curve, t0, t1 })));
            }
        }
        if ring.iter().any(|(p, _)| !p.x.is_finite() || !p.y.is_finite() || p.x.abs() > 1e8 || p.y.abs() > 1e8) {
            return Err("The shape contains invalid coordinates".into());
        }
        ring.dedup_by(|a, b| a.0.distance(b.0) < 1e-9);
        // The last point back on the first closes the ring along the edge that ends at it.
        while ring.len() > 1 && ring.first().zip(ring.last()).is_some_and(|(a, b)| a.0.distance(b.0) < 1e-9) {
            let closing = ring.pop().and_then(|(_, s)| s);
            if let Some(first) = ring.first_mut() {
                first.1 = closing;
            }
        }
        let points: Vec<Point> = ring.iter().map(|(p, _)| *p).collect();
        // Rings with no area are kept here: a contour that crosses itself into lobes that cancel
        // (a bowtie) has none, and [`edges_meet`] must see it. [`orient`] leaves them out. A flat
        // ring, every point on one line, fills nothing under either rule and is left out now, so
        // that it doesn't send the profile through the boolean.
        if points.len() >= 3 && !flat(&points) {
            let n = points.len();
            let spans = (0..n).map(|i| ring.get((i + 1) % n).and_then(|(_, s)| *s)).collect();
            out.push(Ring { points, spans });
        }
    }
    Ok(out)
}

/// `out` (sampled rings) turned so the filled side is on each one's left, and under the nonzero
/// `rule` without the rings that are no edge of the shape, as [`rings`] describes.
fn orient<R: Turnable>(mut out: Vec<R>, rule: FillRule) -> Result<Vec<R>, String> {
    out.retain(|r| signed_area(r.points()).abs() > 1e-9);
    // Which rings hold which: each ring against the others, its edges walked only where its
    // bounds hold the point, and within [`MAX_CONTAINMENT_EDGES`] edges and pairs in all (a pair
    // whose bounds miss the point costs a step too: many small rings are a square of pairs).
    let too_complex = || String::from(TOO_COMPLEX_SHAPE);
    let mut work = 0usize;
    let mut within = |out: &[R], bounds: &[Rect], i: usize| -> Result<Vec<usize>, String> {
        let p = out.get(i).and_then(|r| r.points().first()).copied().unwrap_or_default();
        let mut held_by = vec![];
        for (j, (o, b)) in out.iter().zip(bounds).enumerate() {
            work = work.saturating_add(1);
            if j == i || !(b.x0 <= p.x && p.x <= b.x1 && b.y0 <= p.y && p.y <= b.y1) {
                if work > MAX_CONTAINMENT_EDGES {
                    return Err(too_complex());
                }
                continue;
            }
            work = work.saturating_add(o.points().len());
            if work > MAX_CONTAINMENT_EDGES {
                return Err(too_complex());
            }
            if contains(o.points(), p) {
                held_by.push(j);
            }
        }
        Ok(held_by)
    };
    let bounds_of = |out: &[R]| -> Vec<Rect> {
        out.iter().map(|r| r.points().iter().map(|q| Rect::from_points(*q, *q)).reduce(|a, b| a.union(b)).unwrap_or_default()).collect()
    };
    if rule == FillRule::NonZero {
        let turn = |r: &[Point]| if signed_area(r) > 0.0 { 1 } else { -1 };
        let bounds = bounds_of(&out);
        let mut edge = Vec::with_capacity(out.len());
        for (i, r) in out.iter().enumerate() {
            // The winding number just outside the ring, and just inside it.
            let outside: i32 = within(&out, &bounds, i)?.into_iter().filter_map(|j| out.get(j)).map(|o| turn(o.points())).sum();
            edge.push((outside != 0) != (outside + turn(r.points()) != 0));
        }
        let mut edge = edge.into_iter();
        out.retain(|_| edge.next().unwrap_or(true));
    }
    // A ring inside an odd number of others is a hole.
    let bounds = bounds_of(&out);
    let holes: Vec<bool> = (0..out.len()).map(|i| within(&out, &bounds, i).map(|held| held.len() % 2 == 1)).collect::<Result<_, _>>()?;
    for (r, hole) in out.iter_mut().zip(holes) {
        if (signed_area(r.points()) > 0.0) == hole {
            r.turn_around();
        }
    }
    Ok(out)
}

/// Most ring edges [`rings`] walks to tell which rings hold which: every ring is tested against
/// the others, so many rings of many points each would otherwise take billions of steps.
const MAX_CONTAINMENT_EDGES: usize = 50_000_000;

/// Does every point of `ring` lie on one line (within 1e-9, as [`sample`] merges points)?
fn flat(ring: &[Point]) -> bool {
    let Some(&a) = ring.first() else { return true };
    let Some(&b) = ring.iter().max_by(|p, q| a.distance(**p).total_cmp(&a.distance(**q))) else { return true };
    let len = a.distance(b);
    len <= 1e-9 || ring.iter().all(|p| ((b - a).cross(*p - a) / len).abs() <= 1e-9)
}

pub(super) fn signed_area(r: &[Point]) -> f64 {
    r.iter().zip(r.iter().cycle().skip(1)).take(r.len()).map(|(a, b)| a.x * b.y - b.x * a.y).sum::<f64>() / 2.0
}

pub(super) fn contains(r: &[Point], p: Point) -> bool {
    let mut inside = false;
    for (a, b) in r.iter().zip(r.iter().cycle().skip(1)).take(r.len()) {
        if (a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x) {
            inside = !inside;
        }
    }
    inside
}

/// The unit normal on the left of `a`→`b`: towards the filled side.
pub(super) fn left(a: Point, b: Point) -> (f64, f64) {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let l = dx.hypot(dy);
    if l < 1e-12 { (0.0, 0.0) } else { (-dy / l, dx / l) }
}

/// Most wavefront events processed per pass before pinning the remaining active front.
const MAX_WAVE_EVENTS_MULT: usize = 8;

/// Most candidate collision pairs tested before halting grid hit scans.
const MAX_HIT_WORK: usize = 1 << 24;

/// Grid resolution along the larger axis for wavefront edge/cluster lookup.
const WAVE_GRID_DIM: usize = 64;

/// `rings` offset by each inset distance in `insets` (positive towards the filled side, negative
/// away from it) using an event-driven wavefront simulation so thin parts collapse and opposing
/// fronts stop where they meet instead of crossing and spiking. Subdivides edges in `rings` when a
/// moving vertex stops mid-edge and the endpoints continue past it, keeping partner vertex counts
/// across every step.
pub(super) fn inset_steps(rings: &mut [Ring], insets: &[f64]) -> Vec<Vec<Vec<Point>>> {
    if rings.is_empty() || insets.is_empty() {
        return vec![vec![]; rings.len()];
    }
    if insets.iter().all(|w| *w == 0.0) {
        return rings.iter().map(|r| vec![r.points.clone(); insets.len()]).collect();
    }
    let side = insets.iter().copied().find(|w| *w != 0.0).unwrap_or(0.0);
    let sign = if side >= 0.0 { 1.0 } else { -1.0 };
    let max_tau = insets.iter().map(|w| w.abs()).filter(|w| w.is_finite()).fold(0.0, f64::max);
    if max_tau <= 0.0 {
        return rings.iter().map(|r| vec![r.points.clone(); insets.len()]).collect();
    }
    let sim = simulate_wavefront(rings, insets, sign, max_tau, None);
    if !sim.splits.is_empty()
        && let Some(split_mask) = apply_edge_splits(rings, &sim.splits)
    {
        return simulate_wavefront(rings, insets, sign, max_tau, Some(&split_mask)).steps;
    }
    sim.steps
}

#[derive(Clone)]
struct Cluster {
    orig_pos: Point,
    anchor_pos: Point,
    anchor_tau: f64,
    vel: Vec2,
    in_edge: usize,
    out_edge: usize,
    prev: usize,
    next: usize,
    active: bool,
    pinned: bool,
    split_vertex: bool,
    epoch: u32,
    members: Vec<(usize, usize)>,
}

#[derive(Clone)]
struct WaveEdge {
    ring_idx: usize,
    orig_edge_idx: usize,
    a0: Point,
    b0: Point,
    tangent: Vec2,
    normal: Vec2,
    chamfer: bool,
    head: usize,
    tail: usize,
    active: bool,
    epoch: u32,
}

#[derive(Clone)]
struct SplitCandidate {
    ring_idx: usize,
    orig_edge_idx: usize,
    s: f64,
    hit: Point,
    head: usize,
    tail: usize,
    normal: Vec2,
}

#[derive(Clone, Copy, Debug)]
enum EventKind {
    Collapse { edge: usize, edge_epoch: u32, u: usize, u_epoch: u32, v: usize, v_epoch: u32, w: Option<(usize, u32)> },
    Hit { cluster: usize, c_epoch: u32, edge: usize, edge_epoch: u32, a: usize, a_epoch: u32, b: usize, b_epoch: u32 },
}

#[derive(Clone, Copy, Debug)]
struct QueuedEvent {
    abs_tau: f64,
    is_collapse: bool,
    seq: u64,
    kind: EventKind,
}

impl PartialEq for QueuedEvent {
    fn eq(&self, other: &Self) -> bool {
        self.abs_tau.to_bits() == other.abs_tau.to_bits() && self.is_collapse == other.is_collapse && self.seq == other.seq
    }
}

impl Eq for QueuedEvent {}

impl Ord for QueuedEvent {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.abs_tau.total_cmp(&self.abs_tau).then_with(|| self.is_collapse.cmp(&other.is_collapse)).then_with(|| other.seq.cmp(&self.seq))
    }
}

impl PartialOrd for QueuedEvent {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

struct WaveGrid {
    origin: Point,
    cell: f64,
    n: usize,
    edge_buckets: Vec<Vec<u32>>,
    cluster_buckets: Vec<Vec<u32>>,
    seen_edges: Vec<u32>,
    seen_clusters: Vec<u32>,
    stamp: u32,
    work: usize,
}

impl WaveGrid {
    fn new(clusters: &[Cluster], edges: &[WaveEdge]) -> Self {
        let mut bounds = Rect::ZERO;
        let mut first = true;
        for c in clusters {
            let p = c.orig_pos;
            bounds = if first {
                first = false;
                Rect::from_points(p, p)
            } else {
                bounds.union_pt(p)
            };
        }
        let n = WAVE_GRID_DIM;
        let size = bounds.width().max(bounds.height()).max(1e-6);
        let cell = (size / n as f64).max(1e-6);
        let mut grid = Self {
            origin: bounds.origin(),
            cell,
            n,
            edge_buckets: vec![vec![]; n * n],
            cluster_buckets: vec![vec![]; n * n],
            seen_edges: vec![0; edges.len()],
            seen_clusters: vec![0; clusters.len()],
            stamp: 0,
            work: 0,
        };
        for (e_idx, e) in edges.iter().enumerate() {
            grid.file_edge(e_idx, Rect::from_points(e.a0, e.b0));
        }
        for (c_idx, c) in clusters.iter().enumerate() {
            let (i, j) = grid.cell_coords(c.orig_pos);
            if let Some(bucket) = grid.cluster_buckets.get_mut(j * n + i) {
                bucket.push(c_idx as u32);
            }
        }
        grid
    }

    fn cell_coords(&self, p: Point) -> (usize, usize) {
        let ix = (((p.x - self.origin.x) / self.cell).floor() as isize).clamp(0, self.n as isize - 1) as usize;
        let iy = (((p.y - self.origin.y) / self.cell).floor() as isize).clamp(0, self.n as isize - 1) as usize;
        (ix, iy)
    }

    fn cell_range(&self, r: Rect) -> ((usize, usize), (usize, usize)) {
        let (i0, j0) = self.cell_coords(Point::new(r.x0, r.y0));
        let (i1, j1) = self.cell_coords(Point::new(r.x1, r.y1));
        ((i0.min(i1), i0.max(i1)), (j0.min(j1), j0.max(j1)))
    }

    fn file_edge(&mut self, e_idx: usize, r: Rect) {
        let ((i0, i1), (j0, j1)) = self.cell_range(r);
        let n = self.n;
        let val = e_idx as u32;
        for j in j0..=j1 {
            for i in i0..=i1 {
                if let Some(bucket) = self.edge_buckets.get_mut(j * n + i)
                    && bucket.last() != Some(&val)
                {
                    bucket.push(val);
                }
            }
        }
    }

    fn next_stamp(&mut self) -> u32 {
        self.stamp = self.stamp.wrapping_add(1);
        if self.stamp == 0 {
            self.seen_edges.fill(0);
            self.seen_clusters.fill(0);
            self.stamp = 1;
        }
        self.stamp
    }
}

fn edge_frame(pts: &[Point], i: usize, sign: f64) -> (bool, Vec2, Vec2) {
    let n = pts.len();
    let (Some(&p), Some(&q)) = (pts.get(i), pts.get((i + 1) % n)) else {
        return (true, Vec2::ZERO, Vec2::ZERO);
    };
    let d = q - p;
    let len = d.hypot();
    if len >= 1e-12 {
        let t = d / len;
        return (false, t, Vec2::new(-t.y * sign, t.x * sign));
    }
    let non_zero_normal = |idx: usize| -> Option<Vec2> {
        let (Some(&a), Some(&b)) = (pts.get(idx), pts.get((idx + 1) % n)) else {
            return None;
        };
        let d = b - a;
        let l = d.hypot();
        (l >= 1e-12).then(|| {
            let t = d / l;
            Vec2::new(-t.y * sign, t.x * sign)
        })
    };
    let n_prev = (1..n).find_map(|k| non_zero_normal((i + n - k) % n)).unwrap_or(Vec2::ZERO);
    let n_next = (1..n).find_map(|k| non_zero_normal((i + k) % n)).unwrap_or(Vec2::ZERO);
    let td = n_next - n_prev;
    let tl = td.hypot();
    let t = if tl >= 1e-12 { td / tl } else { Vec2::ZERO };
    let nm = n_prev + n_next;
    let nl = nm.hypot();
    let norm = if nl >= 1e-12 { nm / nl } else { n_prev };
    (true, t, norm)
}

struct Wavefront {
    clusters: Vec<Cluster>,
    edges: Vec<WaveEdge>,
    grid: WaveGrid,
    heap: BinaryHeap<QueuedEvent>,
    seq: u64,
    sign: f64,
    max_tau: f64,
    now_tau: f64,
    split_candidates: Vec<SplitCandidate>,
}

impl Wavefront {
    fn new(rings: &[Ring], sign: f64, max_tau: f64, split_mask: Option<&[Vec<bool>]>) -> Self {
        let total: usize = rings.iter().map(|r| r.points.len()).sum();
        let mut clusters = Vec::with_capacity(total);
        let mut edges = Vec::with_capacity(total);
        for (r_idx, r) in rings.iter().enumerate() {
            let n = r.points.len();
            if n < 3 {
                continue;
            }
            let base_c = clusters.len();
            let base_e = edges.len();
            for i in 0..n {
                let p = r.points[i];
                let q = r.points[(i + 1) % n];
                let (chamfer, tangent, normal) = edge_frame(&r.points, i, sign);
                let split_vertex = split_mask.and_then(|m| m.get(r_idx)).and_then(|v| v.get(i)).copied().unwrap_or(false);
                clusters.push(Cluster {
                    orig_pos: p,
                    anchor_pos: p,
                    anchor_tau: 0.0,
                    vel: Vec2::ZERO,
                    in_edge: base_e + (i + n - 1) % n,
                    out_edge: base_e + i,
                    prev: base_c + (i + n - 1) % n,
                    next: base_c + (i + 1) % n,
                    active: true,
                    pinned: false,
                    split_vertex,
                    epoch: 0,
                    members: vec![(r_idx, i)],
                });
                edges.push(WaveEdge {
                    ring_idx: r_idx,
                    orig_edge_idx: i,
                    a0: p,
                    b0: q,
                    tangent,
                    normal,
                    chamfer,
                    head: base_c + i,
                    tail: base_c + (i + 1) % n,
                    active: true,
                    epoch: 0,
                });
            }
        }
        let grid = WaveGrid::new(&clusters, &edges);
        let mut wf = Self { clusters, edges, grid, heap: BinaryHeap::new(), seq: 0, sign, max_tau, now_tau: 0.0, split_candidates: vec![] };
        for c in 0..wf.clusters.len() {
            wf.update_velocity(c);
        }
        for e in 0..wf.edges.len() {
            wf.enqueue_edge_collapse(e);
        }
        for c in 0..wf.clusters.len() {
            wf.enqueue_cluster_hit(c);
        }
        wf
    }

    fn pos_at(&self, c_idx: usize, t: f64) -> Point {
        let Some(c) = self.clusters.get(c_idx) else {
            return Point::ZERO;
        };
        if c.pinned { c.anchor_pos } else { c.anchor_pos + c.vel * (t - c.anchor_tau).max(0.0) }
    }

    fn pin_cluster(&mut self, c_idx: usize) {
        let p = self.pos_at(c_idx, self.now_tau);
        let Some(c) = self.clusters.get_mut(c_idx) else { return };
        c.anchor_pos = p;
        c.anchor_tau = self.now_tau;
        c.vel = Vec2::ZERO;
        c.pinned = true;
        c.epoch = c.epoch.wrapping_add(1);
    }

    fn corner_velocity(&self, in_e: usize, out_e: usize) -> (Vec2, bool) {
        let (Some(ei), Some(eo)) = (self.edges.get(in_e), self.edges.get(out_e)) else {
            return (Vec2::ZERO, true);
        };
        let ei_zero_chamfer = ei.chamfer && self.pos_at(ei.head, self.now_tau).distance(self.pos_at(ei.tail, self.now_tau)) < 1e-9;
        let eo_zero_chamfer = eo.chamfer && self.pos_at(eo.head, self.now_tau).distance(self.pos_at(eo.tail, self.now_tau)) < 1e-9;
        if eo_zero_chamfer && !ei_zero_chamfer {
            return (ei.normal, false);
        }
        if ei_zero_chamfer && !eo_zero_chamfer {
            return (eo.normal, false);
        }
        let (a, b) = (ei.normal, eo.normal);
        match (a.hypot2() < 1e-18, b.hypot2() < 1e-18) {
            (true, true) => (Vec2::ZERO, true),
            (true, false) => (b, false),
            (false, true) => (a, false),
            (false, false) => {
                let s = a + b;
                let ml = s.hypot();
                if ml < 1e-9 {
                    return (Vec2::ZERO, true);
                }
                let m = s / ml;
                let turn = ei.tangent.cross(eo.tangent);
                let raw_cos = m.dot(a);
                let cos = if turn * self.sign < 0.0 && raw_cos < 1.0 / MITER_LIMIT { 1.0 } else { raw_cos.max(1.0 / MITER_LIMIT) };
                (m * (1.0 / cos), false)
            }
        }
    }

    fn update_velocity(&mut self, c_idx: usize) {
        let Some(c) = self.clusters.get(c_idx) else { return };
        if !c.active || c.pinned {
            return;
        }
        if c.next == c_idx || self.clusters.get(c.next).is_some_and(|n| n.next == c_idx) {
            self.retire_two_vertex_loop(c_idx);
            return;
        }
        let (vel, should_pin) = self.corner_velocity(c.in_edge, c.out_edge);
        let p = self.pos_at(c_idx, self.now_tau);
        if let Some(c) = self.clusters.get_mut(c_idx) {
            c.anchor_pos = p;
            c.anchor_tau = self.now_tau;
            c.vel = vel;
            if should_pin {
                c.pinned = true;
            }
            c.epoch = c.epoch.wrapping_add(1);
        }
    }

    fn retire_loop_edges(&mut self, idxs: &[usize]) {
        for &idx in idxs {
            if let Some(cl) = self.clusters.get(idx).cloned() {
                for e_idx in [cl.in_edge, cl.out_edge] {
                    if let Some(e) = self.edges.get_mut(e_idx) {
                        e.active = false;
                        e.epoch = e.epoch.wrapping_add(1);
                    }
                }
            }
        }
    }

    fn retire_two_vertex_loop(&mut self, c_idx: usize) {
        let Some(other) = self.clusters.get(c_idx).map(|c| c.next) else { return };
        self.pin_cluster(c_idx);
        if other != c_idx {
            self.pin_cluster(other);
        }
        self.retire_loop_edges(&[c_idx, other]);
    }

    fn retire_three_vertex_loop(&mut self, a: usize, b: usize, c: usize) {
        for idx in [a, b, c] {
            self.pin_cluster(idx);
        }
        self.retire_loop_edges(&[a, b, c]);
    }

    fn push_event(&mut self, abs_tau: f64, is_collapse: bool, kind: EventKind) {
        if !abs_tau.is_finite() || abs_tau < self.now_tau || abs_tau > self.max_tau + 1e-12 {
            return;
        }
        self.seq = self.seq.wrapping_add(1);
        self.heap.push(QueuedEvent { abs_tau: abs_tau.min(self.max_tau), is_collapse, seq: self.seq, kind });
    }

    fn enqueue_edge_collapse(&mut self, e_idx: usize) {
        let Some(e1) = self.edges.get(e_idx).cloned() else { return };
        if !e1.active || e1.head == e1.tail {
            return;
        }
        let (u, v) = (e1.head, e1.tail);
        let Some((vu, u_epoch)) = self.clusters.get(u).filter(|c| c.active).map(|c| (c.vel, c.epoch)) else { return };
        let Some((vv, v_epoch, e2_idx, u_pinned, v_pinned)) = self
            .clusters
            .get(u)
            .zip(self.clusters.get(v))
            .and_then(|(cu, cv)| (cv.active && !(cu.pinned && cv.pinned)).then_some((cv.vel, cv.epoch, cv.out_edge, cu.pinned, cv.pinned)))
        else {
            return;
        };
        let _ = (u_pinned, v_pinned);
        let t0 = self.now_tau;
        let rem = self.max_tau - t0;
        let (pu, pv) = (self.pos_at(u, t0), self.pos_at(v, t0));

        let len = (pv - pu).dot(e1.tangent);
        let dlen = (vv - vu).dot(e1.tangent);
        let len_dt = if !e1.chamfer && (len < -1e-9 || (len <= 1e-9 && dlen <= 1e-9)) {
            Some(0.0)
        } else if (!e1.chamfer || len > 1e-9) && dlen < -1e-12 {
            let dt = (len / -dlen).max(0.0);
            (dt <= rem + 1e-12).then_some(dt)
        } else {
            None
        };
        if let Some(dt) = len_dt {
            self.push_event(t0 + dt, true, EventKind::Collapse { edge: e_idx, edge_epoch: e1.epoch, u, u_epoch, v, v_epoch, w: None });
        }

        let Some(e2) = self.edges.get(e2_idx).cloned() else { return };
        let w = e2.tail;
        let Some((vw, w_epoch)) = self.clusters.get(w).filter(|c| c.active).map(|c| (c.vel, c.epoch)) else { return };
        if !e2.active || w == u || e1.chamfer || e2.chamfer {
            return;
        }
        let pw = self.pos_at(w, t0);
        let (p0, dp) = (pv - pu, vv - vu);
        let (q0, dq) = (pw - pv, vw - vv);
        let c_q = self.sign * p0.cross(q0);
        let b_q = self.sign * (p0.cross(dq) + dp.cross(q0));
        let a_q = self.sign * dp.cross(dq);
        let turn0 = e1.tangent.cross(e2.tangent) * self.sign;
        let scale = (p0.hypot() * q0.hypot()).max(1.0);
        let tri_dt = if c_q <= 1e-9 * scale {
            (c_q >= -1e-9 * scale && turn0 >= -1e-9 && b_q < -1e-12 * scale).then_some(0.0)
        } else {
            smallest_crossing_root(a_q, b_q, c_q, rem, 1e-12 * scale)
        };
        if let Some(dt) = tri_dt {
            let d1 = (pu + vu * dt).distance(pv + vv * dt);
            let d2 = (pv + vv * dt).distance(pw + vw * dt);
            let (pick_idx, pick) = if d1 <= d2 { (e_idx, &e1) } else { (e2_idx, &e2) };
            let (pu_epoch, pv_epoch) = (self.clusters.get(pick.head).map_or(0, |c| c.epoch), self.clusters.get(pick.tail).map_or(0, |c| c.epoch));
            let third = if d1 <= d2 { (w, w_epoch) } else { (u, u_epoch) };
            self.push_event(
                t0 + dt,
                true,
                EventKind::Collapse {
                    edge: pick_idx,
                    edge_epoch: pick.epoch,
                    u: pick.head,
                    u_epoch: pu_epoch,
                    v: pick.tail,
                    v_epoch: pv_epoch,
                    w: Some(third),
                },
            );
        }
    }

    fn hit_time(&self, c_idx: usize, e_idx: usize, max_dt: f64) -> Option<f64> {
        let (Some(cc), Some(e)) = (self.clusters.get(c_idx), self.edges.get(e_idx)) else {
            return None;
        };
        if !cc.active || cc.pinned || !e.active {
            return None;
        }
        let (a, b) = (e.head, e.tail);
        if a == c_idx || b == c_idx || b == cc.prev || a == cc.next {
            return None;
        }
        let (Some(ca), Some(cb)) = (self.clusters.get(a), self.clusters.get(b)) else {
            return None;
        };
        if !ca.active || !cb.active {
            return None;
        }
        let (vc, va, vb) = (cc.vel, ca.vel, cb.vel);
        if vc.dot(e.normal) >= va.dot(e.normal).max(vb.dot(e.normal)) - 1e-9 {
            return None;
        }
        let t0 = self.now_tau;
        let (pc, pa, pb) = (self.pos_at(c_idx, t0), self.pos_at(a, t0), self.pos_at(b, t0));
        if !boxes_overlap_during(pc, vc, pa, va, pb, vb, max_dt) {
            return None;
        }
        let (u0, du) = (pb - pa, vb - va);
        let (v0, dv) = (pc - pa, vc - va);
        let c_q = self.sign * u0.cross(v0);
        let b_q = self.sign * (u0.cross(dv) + du.cross(v0));
        let a_q = self.sign * du.cross(dv);
        let len0 = u0.hypot();
        let dt = if len0 < 1e-9 {
            if pc.distance(pa) <= 1e-5 && b_q < -1e-9 {
                Some(0.0)
            } else if b_q > 1e-9 && a_q < -1e-9 {
                let r = -b_q / a_q;
                (r >= 0.0 && r <= max_dt).then_some(r)
            } else {
                None
            }
        } else if c_q.abs() <= 1e-5 * len0 && b_q < -1e-9 * len0 {
            Some(0.0)
        } else if c_q > 0.0 {
            smallest_crossing_root(a_q, b_q, c_q, max_dt, 1e-9 * len0)
        } else {
            None
        }?;
        let (pa_t, pb_t, pc_t) = (pa + va * dt, pb + vb * dt, pc + vc * dt);
        let ab_t = pb_t - pa_t;
        let len_t = ab_t.hypot();
        let on_seg = if len_t > 1e-9 {
            let proj = (pc_t - pa_t).dot(ab_t) / len_t;
            proj >= -1e-5 && proj <= len_t + 1e-5
        } else {
            pc_t.distance(pa_t) <= 1e-5
        };
        on_seg.then_some(dt)
    }

    fn candidate_edges_for(&mut self, search: Rect) -> Vec<usize> {
        let ((i0, i1), (j0, j1)) = self.grid.cell_range(search);
        let cells = (i1 - i0 + 1) * (j1 - j0 + 1);
        if self.edges.len() <= 256 || cells >= self.grid.n * self.grid.n / 2 {
            return (0..self.edges.len()).collect();
        }
        let stamp = self.grid.next_stamp();
        let n = self.grid.n;
        let mut out = vec![];
        for j in j0..=j1 {
            for i in i0..=i1 {
                let len = self.grid.edge_buckets.get(j * n + i).map_or(0, Vec::len);
                for k in 0..len {
                    let e_idx = self.grid.edge_buckets[j * n + i][k] as usize;
                    if self.grid.seen_edges.get(e_idx) != Some(&stamp) {
                        if let Some(slot) = self.grid.seen_edges.get_mut(e_idx) {
                            *slot = stamp;
                        }
                        out.push(e_idx);
                    }
                }
            }
        }
        out
    }

    fn candidate_clusters_for(&mut self, search: Rect) -> Vec<usize> {
        let ((i0, i1), (j0, j1)) = self.grid.cell_range(search);
        let cells = (i1 - i0 + 1) * (j1 - j0 + 1);
        if self.clusters.len() <= 256 || cells >= self.grid.n * self.grid.n / 2 {
            return (0..self.clusters.len()).collect();
        }
        let stamp = self.grid.next_stamp();
        let n = self.grid.n;
        let mut out = vec![];
        for j in j0..=j1 {
            for i in i0..=i1 {
                let len = self.grid.cluster_buckets.get(j * n + i).map_or(0, Vec::len);
                for k in 0..len {
                    let c_idx = self.grid.cluster_buckets[j * n + i][k] as usize;
                    if self.grid.seen_clusters.get(c_idx) != Some(&stamp) {
                        if let Some(slot) = self.grid.seen_clusters.get_mut(c_idx) {
                            *slot = stamp;
                        }
                        out.push(c_idx);
                    }
                }
            }
        }
        out
    }

    fn push_hit_if_valid(&mut self, c_idx: usize, e_idx: usize, abs_tau: f64) {
        if let (Some(cc), Some(e)) = (self.clusters.get(c_idx), self.edges.get(e_idx))
            && let (Some(ca), Some(cb)) = (self.clusters.get(e.head), self.clusters.get(e.tail))
        {
            self.push_event(
                abs_tau,
                false,
                EventKind::Hit {
                    cluster: c_idx,
                    c_epoch: cc.epoch,
                    edge: e_idx,
                    edge_epoch: e.epoch,
                    a: e.head,
                    a_epoch: ca.epoch,
                    b: e.tail,
                    b_epoch: cb.epoch,
                },
            );
        }
    }

    fn enqueue_cluster_hit(&mut self, c_idx: usize) {
        if self.grid.work > MAX_HIT_WORK {
            return;
        }
        let Some(cc) = self.clusters.get(c_idx) else { return };
        if !cc.active || cc.pinned {
            return;
        }
        let t0 = self.now_tau;
        let rem = self.max_tau - t0;
        let p0 = self.pos_at(c_idx, t0);
        let q = Rect::from_points(p0, p0 + cc.vel * rem);
        let reach = MITER_LIMIT * self.max_tau + 1.0;
        let search = Rect::new(q.x0 - reach, q.y0 - reach, q.x1 + reach, q.y1 + reach);
        let candidates = self.candidate_edges_for(search);
        let mut best: Option<(f64, usize)> = None;
        for e_idx in candidates {
            self.grid.work = self.grid.work.saturating_add(1);
            let max_dt = best.map_or(rem, |(d, _)| d);
            if let Some(dt) = self.hit_time(c_idx, e_idx, max_dt) {
                best = Some((dt, e_idx));
            }
        }
        if let Some((dt, e_idx)) = best {
            self.push_hit_if_valid(c_idx, e_idx, t0 + dt);
        }
    }

    fn enqueue_hits_against_edge(&mut self, e_idx: usize) {
        if self.grid.work > MAX_HIT_WORK {
            return;
        }
        let Some(e) = self.edges.get(e_idx).cloned() else { return };
        if !e.active {
            return;
        }
        let (Some(ca), Some(cb)) = (self.clusters.get(e.head), self.clusters.get(e.tail)) else { return };
        if !ca.active || !cb.active {
            return;
        }
        let t0 = self.now_tau;
        let rem = self.max_tau - t0;
        let (pa0, pb0) = (self.pos_at(e.head, t0), self.pos_at(e.tail, t0));
        let r = Rect::from_points(pa0, pb0).union_pt(pa0 + ca.vel * rem).union_pt(pb0 + cb.vel * rem);
        let reach = MITER_LIMIT * self.max_tau + 1.0;
        let search = Rect::new(r.x0 - reach, r.y0 - reach, r.x1 + reach, r.y1 + reach);
        let candidates = self.candidate_clusters_for(search);
        for c_idx in candidates {
            self.grid.work = self.grid.work.saturating_add(1);
            if let Some(dt) = self.hit_time(c_idx, e_idx, rem) {
                self.push_hit_if_valid(c_idx, e_idx, t0 + dt);
            }
        }
    }

    fn refresh_around(&mut self, c_idx: usize) {
        let Some(c) = self.clusters.get(c_idx).cloned() else { return };
        if let Some(prev_in) = self.clusters.get(c.prev).map(|p| p.in_edge) {
            self.enqueue_edge_collapse(prev_in);
        }
        self.enqueue_edge_collapse(c.in_edge);
        self.enqueue_edge_collapse(c.out_edge);
        if c.active && !c.pinned {
            self.enqueue_cluster_hit(c_idx);
        }
        self.enqueue_hits_against_edge(c.in_edge);
        self.enqueue_hits_against_edge(c.out_edge);
    }

    fn refile_edge(&mut self, e_idx: usize) {
        let Some(e) = self.edges.get(e_idx) else { return };
        let (pa, pb) = (self.pos_at(e.head, self.now_tau), self.pos_at(e.tail, self.now_tau));
        self.grid.file_edge(e_idx, Rect::from_points(pa, pb));
    }

    fn event_valid(&self, ev: &QueuedEvent) -> EventStatus {
        match ev.kind {
            EventKind::Collapse { edge, edge_epoch, u, u_epoch, v, v_epoch, w } => {
                let (Some(e), Some(cu), Some(cv)) = (self.edges.get(edge), self.clusters.get(u), self.clusters.get(v)) else {
                    return EventStatus::Stale;
                };
                if !e.active || e.epoch != edge_epoch || !cu.active || cu.epoch != u_epoch || !cv.active || cv.epoch != v_epoch {
                    return EventStatus::Stale;
                }
                if let Some((w_idx, w_epoch)) = w {
                    let Some(cw) = self.clusters.get(w_idx) else { return EventStatus::Stale };
                    if !cw.active || cw.epoch != w_epoch {
                        return EventStatus::Stale;
                    }
                }
                EventStatus::Valid
            }
            EventKind::Hit { cluster, c_epoch, edge, edge_epoch, a, a_epoch, b, b_epoch } => {
                let Some(cc) = self.clusters.get(cluster) else { return EventStatus::Stale };
                if !cc.active || cc.pinned || cc.epoch != c_epoch {
                    return EventStatus::Stale;
                }
                let (Some(e), Some(ca), Some(cb)) = (self.edges.get(edge), self.clusters.get(a), self.clusters.get(b)) else {
                    return EventStatus::RequeryCluster(cluster);
                };
                if !e.active || e.epoch != edge_epoch || !ca.active || ca.epoch != a_epoch || !cb.active || cb.epoch != b_epoch {
                    return EventStatus::RequeryCluster(cluster);
                }
                EventStatus::Valid
            }
        }
    }

    fn merge_collapsed_edge(&mut self, e_idx: usize, u: usize, v: usize, pu: Point, pv: Point) {
        let (Some(cu), Some(cv)) = (self.clusters.get(u).cloned(), self.clusters.get(v).cloned()) else { return };
        if let Some(edge) = self.edges.get_mut(e_idx) {
            edge.active = false;
            edge.epoch = edge.epoch.wrapping_add(1);
        }
        let merged_pos = if cu.pinned && !cv.pinned {
            pu
        } else if cv.pinned && !cu.pinned {
            pv
        } else {
            pu.midpoint(pv)
        };
        let (out_e, nxt) = (cv.out_edge, cv.next);
        if let Some(edge) = self.edges.get_mut(out_e) {
            edge.head = u;
            edge.epoch = edge.epoch.wrapping_add(1);
        }
        if let Some(c_u) = self.clusters.get_mut(u) {
            c_u.anchor_pos = merged_pos;
            c_u.anchor_tau = self.now_tau;
            c_u.pinned = cu.pinned || cv.pinned;
            c_u.out_edge = out_e;
            c_u.next = nxt;
            c_u.epoch = c_u.epoch.wrapping_add(1);
        }
        if let Some(c_nxt) = self.clusters.get_mut(nxt) {
            c_nxt.prev = u;
        }
        if let Some(c_v) = self.clusters.get_mut(v) {
            c_v.active = false;
            c_v.vel = Vec2::ZERO;
            c_v.epoch = c_v.epoch.wrapping_add(1);
        }
        let v_members = std::mem::take(&mut self.clusters[v].members);
        self.clusters[u].members.extend(v_members);
        self.update_velocity(u);
        self.refile_edge(out_e);
        self.refresh_around(u);
    }

    fn collapse_lagging_corner(&mut self, e_idx: usize, u: usize, v: usize, pin_v: bool) {
        let (Some(cu), Some(cv)) = (self.clusters.get(u).cloned(), self.clusters.get(v).cloned()) else { return };
        if let Some(edge) = self.edges.get_mut(e_idx) {
            edge.active = false;
            edge.epoch = edge.epoch.wrapping_add(1);
        }
        if pin_v {
            let (out_e, nxt) = (cv.out_edge, cv.next);
            self.pin_cluster(v);
            if let Some(c_v) = self.clusters.get_mut(v) {
                c_v.active = false;
                c_v.epoch = c_v.epoch.wrapping_add(1);
            }
            if let Some(edge) = self.edges.get_mut(out_e) {
                edge.head = u;
                edge.epoch = edge.epoch.wrapping_add(1);
            }
            if let Some(c_u) = self.clusters.get_mut(u) {
                c_u.out_edge = out_e;
                c_u.next = nxt;
                c_u.epoch = c_u.epoch.wrapping_add(1);
            }
            if let Some(c_nxt) = self.clusters.get_mut(nxt) {
                c_nxt.prev = u;
            }
            self.update_velocity(u);
            self.refile_edge(out_e);
            self.refresh_around(u);
            self.enqueue_collapsed_shortcut(out_e);
        } else {
            let (in_e, prv) = (cu.in_edge, cu.prev);
            self.pin_cluster(u);
            if let Some(c_u) = self.clusters.get_mut(u) {
                c_u.active = false;
                c_u.epoch = c_u.epoch.wrapping_add(1);
            }
            if let Some(edge) = self.edges.get_mut(in_e) {
                edge.tail = v;
                edge.epoch = edge.epoch.wrapping_add(1);
            }
            if let Some(c_v) = self.clusters.get_mut(v) {
                c_v.in_edge = in_e;
                c_v.prev = prv;
                c_v.epoch = c_v.epoch.wrapping_add(1);
            }
            if let Some(c_prv) = self.clusters.get_mut(prv) {
                c_prv.next = v;
            }
            self.update_velocity(v);
            self.refile_edge(in_e);
            self.refresh_around(v);
            self.enqueue_collapsed_shortcut(in_e);
        }
    }

    fn enqueue_collapsed_shortcut(&mut self, e_idx: usize) {
        let Some(e) = self.edges.get(e_idx).cloned() else { return };
        if !e.active || e.head == e.tail || e.chamfer {
            return;
        }
        let (Some(cu), Some(cv)) = (self.clusters.get(e.head), self.clusters.get(e.tail)) else { return };
        if !cu.active || !cv.active {
            return;
        }
        let (pu, pv) = (self.pos_at(e.head, self.now_tau), self.pos_at(e.tail, self.now_tau));
        if pu.distance(pv) <= 1e-4 || (pv - pu).dot(e.tangent) <= 1e-6 {
            self.push_event(
                self.now_tau,
                true,
                EventKind::Collapse { edge: e_idx, edge_epoch: e.epoch, u: e.head, u_epoch: cu.epoch, v: e.tail, v_epoch: cv.epoch, w: None },
            );
        }
    }

    fn execute_collapse(&mut self, e_idx: usize) {
        let Some(e) = self.edges.get(e_idx).cloned() else { return };
        let (u, v) = (e.head, e.tail);
        let (Some(cu), Some(cv)) = (self.clusters.get(u), self.clusters.get(v)) else { return };
        if u == v || cv.next == u || cu.prev == v {
            self.retire_two_vertex_loop(u);
            return;
        }
        let (pu, pv) = (self.pos_at(u, self.now_tau), self.pos_at(v, self.now_tau));
        if pu.distance(pv) <= 1e-4 {
            self.merge_collapsed_edge(e_idx, u, v, pu, pv);
            return;
        }
        if let Some(nxt_nxt) = self.clusters.get(cv.next).map(|cn| cn.next)
            && nxt_nxt == u
        {
            self.retire_three_vertex_loop(u, v, cv.next);
            return;
        }
        let du = (pu - e.a0).dot(e.normal);
        let dv = (pv - e.a0).dot(e.normal);
        // When u is already pinned, v is the moving vertex colliding into e; otherwise pin whichever
        // endpoint lagged behind (or v on a tie).
        let pin_v = cu.pinned || cv.pinned || du >= dv - 1e-4;
        self.collapse_lagging_corner(e_idx, u, v, pin_v);
    }

    fn execute_hit(&mut self, c_idx: usize, e_idx: usize) {
        let hit = self.pos_at(c_idx, self.now_tau);
        self.pin_cluster(c_idx);
        let Some(e) = self.edges.get(e_idx).cloned() else { return };
        let (a, b) = (e.head, e.tail);
        let (pa, pb) = (self.pos_at(a, self.now_tau), self.pos_at(b, self.now_tau));
        let thr_a = if self.clusters.get(a).is_some_and(|c| c.split_vertex) { 0.5 } else { 0.1 };
        let thr_b = if self.clusters.get(b).is_some_and(|c| c.split_vertex) { 0.5 } else { 0.1 };
        let a_hit = hit.distance(pa) <= thr_a;
        let b_hit = hit.distance(pb) <= thr_b;
        if a_hit && self.clusters.get(a).is_some_and(|c| !c.pinned) {
            self.pin_cluster(a);
            self.refresh_around(a);
        }
        if b_hit && self.clusters.get(b).is_some_and(|c| !c.pinned) {
            self.pin_cluster(b);
            self.refresh_around(b);
        }
        if !a_hit && !b_hit && !e.chamfer {
            let len0 = e.a0.distance(e.b0);
            if len0 > 1e-9 {
                let s = ((hit - e.normal * self.now_tau) - e.a0).dot(e.tangent) / len0;
                if s > 0.05 / len0 && s < 1.0 - 0.05 / len0 {
                    self.split_candidates.push(SplitCandidate {
                        ring_idx: e.ring_idx,
                        orig_edge_idx: e.orig_edge_idx,
                        s,
                        hit,
                        head: a,
                        tail: b,
                        normal: e.normal,
                    });
                }
            }
        }
        self.refresh_around(c_idx);
    }

    fn check_pinned_sweep(&mut self, c_idx: usize, out: &mut Vec<EdgeSplit>) {
        let Some(cc) = self.clusters.get(c_idx) else { return };
        if !cc.pinned {
            return;
        }
        let (pc, orig_c) = (cc.anchor_pos, cc.orig_pos);
        let reach = MITER_LIMIT * self.max_tau + 1.0;
        let search = Rect::new(pc.x - reach, pc.y - reach, pc.x + reach, pc.y + reach);
        let edges = self.candidate_edges_for(search);
        for e_idx in edges {
            let Some(e) = self.edges.get(e_idx) else { continue };
            if !e.active || e.chamfer || e.head == c_idx || e.tail == c_idx || (orig_c - e.a0).dot(e.normal) <= 0.0 {
                continue;
            }
            let len0 = e.a0.distance(e.b0);
            if len0 <= 0.2 {
                continue;
            }
            let s = (pc - e.a0).dot(e.tangent) / len0;
            if s <= 0.1 / len0 || s >= 1.0 - 0.1 / len0 {
                continue;
            }
            let (pa, pb) = (self.pos_at(e.head, self.max_tau), self.pos_at(e.tail, self.max_tau));
            let ab = pb - pa;
            let ab2 = ab.hypot2();
            if ab2 < 1e-18 {
                continue;
            }
            let t_cur = (pc - pa).dot(ab) / ab2;
            if (0.0..=1.0).contains(&t_cur) && (pa.lerp(pb, t_cur) - pc).dot(e.normal) > 0.05 {
                out.push(EdgeSplit { ring_idx: e.ring_idx, edge_idx: e.orig_edge_idx, s });
            }
        }
    }

    fn needed_splits(&mut self) -> Vec<EdgeSplit> {
        let mut out = vec![];
        for cand in &self.split_candidates {
            let p_head = self.pos_at(cand.head, self.max_tau);
            let p_tail = self.pos_at(cand.tail, self.max_tau);
            if (p_head - cand.hit).dot(cand.normal) > 0.05 || (p_tail - cand.hit).dot(cand.normal) > 0.05 {
                out.push(EdgeSplit { ring_idx: cand.ring_idx, edge_idx: cand.orig_edge_idx, s: cand.s });
            }
        }
        for c_idx in 0..self.clusters.len() {
            self.check_pinned_sweep(c_idx, &mut out);
        }
        out
    }

    fn snapshot(&self, rings: &[Ring], t: f64) -> Vec<Vec<Point>> {
        let mut out: Vec<Vec<Point>> = rings.iter().map(|r| r.points.clone()).collect();
        for c_idx in 0..self.clusters.len() {
            let p = self.pos_at(c_idx, t);
            if let Some(c) = self.clusters.get(c_idx) {
                for &(r_idx, v_idx) in &c.members {
                    if let Some(slot) = out.get_mut(r_idx).and_then(|ring| ring.get_mut(v_idx)) {
                        *slot = p;
                    }
                }
            }
        }
        out
    }
}

enum EventStatus {
    Valid,
    Stale,
    RequeryCluster(usize),
}

fn boxes_overlap_during(pc: Point, vc: Vec2, pa: Point, va: Vec2, pb: Point, vb: Vec2, max_dt: f64) -> bool {
    let (cx0, cx1) = (pc.x.min(pc.x + vc.x * max_dt), pc.x.max(pc.x + vc.x * max_dt));
    let (ex0, ex1) =
        (pa.x.min(pb.x).min(pa.x + va.x * max_dt).min(pb.x + vb.x * max_dt), pa.x.max(pb.x).max(pa.x + va.x * max_dt).max(pb.x + vb.x * max_dt));
    if cx1 + 1e-4 < ex0 || ex1 + 1e-4 < cx0 {
        return false;
    }
    let (cy0, cy1) = (pc.y.min(pc.y + vc.y * max_dt), pc.y.max(pc.y + vc.y * max_dt));
    let (ey0, ey1) =
        (pa.y.min(pb.y).min(pa.y + va.y * max_dt).min(pb.y + vb.y * max_dt), pa.y.max(pb.y).max(pa.y + va.y * max_dt).max(pb.y + vb.y * max_dt));
    !(cy1 + 1e-4 < ey0 || ey1 + 1e-4 < cy0)
}

fn smallest_crossing_root(a_q: f64, b_q: f64, c_q: f64, max_dt: f64, deriv_tol: f64) -> Option<f64> {
    if a_q.abs() <= 1e-12 * (b_q.abs() + c_q.abs() + 1.0) {
        if b_q < -deriv_tol {
            let t = -c_q / b_q;
            if t >= 0.0 && t <= max_dt + 1e-12 {
                return Some(t.clamp(0.0, max_dt));
            }
        }
        return None;
    }
    let disc = b_q * b_q - 4.0 * a_q * c_q;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let q = -0.5 * (b_q + b_q.signum() * s);
    let mut roots = [f64::INFINITY; 2];
    roots[0] = q / a_q;
    if q.abs() > 1e-300 {
        roots[1] = c_q / q;
    }
    roots.sort_by(f64::total_cmp);
    for t in roots {
        if t.is_finite() && t >= -1e-12 && t <= max_dt + 1e-12 && (2.0 * a_q * t.max(0.0) + b_q) < -deriv_tol {
            return Some(t.clamp(0.0, max_dt));
        }
    }
    None
}

fn record_step(out: &mut [Vec<Vec<Point>>], rings: &[Ring], wf: &Wavefront, s_idx: usize, s_tau: f64, prev: &mut Option<(f64, usize)>) {
    if let Some((ptau, ps)) = *prev
        && ptau == s_tau
    {
        for ring_steps in out.iter_mut() {
            if let Some(existing) = ring_steps.get(ps).cloned()
                && let Some(slot) = ring_steps.get_mut(s_idx)
            {
                *slot = existing;
            }
        }
        return;
    }
    let snap = wf.snapshot(rings, s_tau);
    for (r_idx, pts) in snap.into_iter().enumerate() {
        if let Some(slot) = out.get_mut(r_idx).and_then(|s| s.get_mut(s_idx)) {
            *slot = pts;
        }
    }
    *prev = Some((s_tau, s_idx));
}

struct EdgeSplit {
    ring_idx: usize,
    edge_idx: usize,
    s: f64,
}

struct SimResult {
    steps: Vec<Vec<Vec<Point>>>,
    splits: Vec<EdgeSplit>,
}

fn simulate_wavefront(rings: &[Ring], insets: &[f64], sign: f64, max_tau: f64, split_mask: Option<&[Vec<bool>]>) -> SimResult {
    let second_pass = split_mask.is_some();
    let mut wf = Wavefront::new(rings, sign, max_tau, split_mask);
    let mut out: Vec<Vec<Vec<Point>>> = rings.iter().map(|r| vec![r.points.clone(); insets.len()]).collect();
    let mut order: Vec<usize> = (0..insets.len()).collect();
    order.sort_by(|&a, &b| insets[a].abs().total_cmp(&insets[b].abs()));
    let mut cursor = 0usize;
    let mut prev: Option<(f64, usize)> = None;

    while cursor < order.len() && insets[order[cursor]].abs() <= 0.0 {
        prev = Some((0.0, order[cursor]));
        cursor += 1;
    }

    let max_events = MAX_WAVE_EVENTS_MULT.saturating_mul(wf.clusters.len().saturating_add(wf.edges.len())).max(64);
    let mut handled = 0usize;
    let mut halted = false;
    while cursor < order.len() || (!second_pass && !wf.heap.is_empty()) {
        let Some(ev) = wf.heap.pop() else {
            break;
        };
        match wf.event_valid(&ev) {
            EventStatus::Stale => continue,
            EventStatus::RequeryCluster(c_idx) => {
                wf.enqueue_cluster_hit(c_idx);
                continue;
            }
            EventStatus::Valid => {}
        }
        if ev.abs_tau > max_tau + 1e-12 {
            break;
        }
        while cursor < order.len() && insets[order[cursor]].abs() <= ev.abs_tau {
            let s_idx = order[cursor];
            let s_tau = insets[s_idx].abs().clamp(0.0, max_tau);
            record_step(&mut out, rings, &wf, s_idx, s_tau, &mut prev);
            cursor += 1;
        }
        if ev.abs_tau > wf.now_tau {
            wf.now_tau = ev.abs_tau.min(max_tau);
        }
        match ev.kind {
            EventKind::Collapse { edge, .. } => wf.execute_collapse(edge),
            EventKind::Hit { cluster, edge, .. } => wf.execute_hit(cluster, edge),
        }
        handled += 1;
        if handled >= max_events {
            halted = true;
            break;
        }
    }

    while cursor < order.len() {
        let s_idx = order[cursor];
        let s_tau = if halted { wf.now_tau } else { insets[s_idx].abs().clamp(0.0, max_tau) };
        record_step(&mut out, rings, &wf, s_idx, s_tau, &mut prev);
        cursor += 1;
    }

    let splits = if second_pass { vec![] } else { wf.needed_splits() };
    SimResult { steps: out, splits }
}

fn apply_edge_splits(rings: &mut [Ring], splits: &[EdgeSplit]) -> Option<Vec<Vec<bool>>> {
    let mut by_ring: Vec<Vec<Vec<f64>>> = rings.iter().map(|r| vec![vec![]; r.points.len()]).collect();
    for split in splits {
        if let Some(slot) = by_ring.get_mut(split.ring_idx).and_then(|e| e.get_mut(split.edge_idx))
            && split.s.is_finite()
        {
            slot.push(split.s);
        }
    }
    let mut total_added = 0usize;
    let total_points: usize = rings.iter().map(|r| r.points.len()).sum();
    for (r_idx, r) in rings.iter().enumerate() {
        let n = r.points.len();
        if n < 3 {
            continue;
        }
        for (e_idx, edge_splits) in by_ring[r_idx].iter_mut().enumerate() {
            if edge_splits.is_empty() {
                continue;
            }
            let len = r.points[e_idx].distance(r.points[(e_idx + 1) % n]);
            if len <= 0.1 {
                edge_splits.clear();
                continue;
            }
            let min_ds = 0.05 / len;
            edge_splits.sort_by(f64::total_cmp);
            edge_splits.retain(|&s| s > min_ds && s < 1.0 - min_ds);
            let mut deduped: Vec<f64> = Vec::with_capacity(edge_splits.len());
            for &s in edge_splits.iter() {
                if deduped.last().is_none_or(|&prev| s - prev >= min_ds) {
                    deduped.push(s);
                }
            }
            total_added = total_added.saturating_add(deduped.len());
            *edge_splits = deduped;
        }
    }
    if total_added == 0 || total_points.saturating_add(total_added) > MAX_SOLID_POINTS {
        return None;
    }
    let mut split_mask = Vec::with_capacity(rings.len());
    for (r_idx, r) in rings.iter_mut().enumerate() {
        let n = r.points.len();
        if n < 3 || by_ring[r_idx].iter().all(Vec::is_empty) {
            split_mask.push(vec![false; n]);
            continue;
        }
        let extra: usize = by_ring[r_idx].iter().map(Vec::len).sum();
        let mut new_points = Vec::with_capacity(n + extra);
        let mut new_spans = Vec::with_capacity(n + extra);
        let mut mask = Vec::with_capacity(n + extra);
        for i in 0..n {
            let p0 = r.points[i];
            let p1 = r.points[(i + 1) % n];
            let span = r.spans.get(i).copied().flatten();
            new_points.push(p0);
            mask.push(false);
            let mut prev_s = 0.0;
            for &s in &by_ring[r_idx][i] {
                new_points.push(p0.lerp(p1, s));
                mask.push(true);
                new_spans.push(span.map(|sp| Span { curve: sp.curve, t0: sp.t0 + (sp.t1 - sp.t0) * prev_s, t1: sp.t0 + (sp.t1 - sp.t0) * s }));
                prev_s = s;
            }
            new_spans.push(span.map(|sp| Span { curve: sp.curve, t0: sp.t0 + (sp.t1 - sp.t0) * prev_s, t1: sp.t1 }));
        }
        r.points = new_points;
        r.spans = new_spans;
        split_mask.push(mask);
    }
    Some(split_mask)
}

/// `ring` with each corner whose miter an inset towards `side` (positive: the filled side,
/// negative: away from it) would reach past [`MITER_LIMIT`] given twice, joined by an edge of no
/// length that follows no curve. [`inset_steps`] moves the two copies apart along their own edges'
/// normals, so the inset ring cuts the corner square, as a bevel join does, instead of running out
/// to a point along the bisector. Corners on the other side of the turn, whose inset edges meet
/// before the corner, keep one point.
pub(super) fn square_corners(ring: &Ring, side: f64) -> Ring {
    let n = ring.points.len();
    if side == 0.0 || n < 3 || ring.spans.len() != n {
        return ring.clone();
    }
    let mut out = Ring { points: Vec::with_capacity(n), spans: Vec::with_capacity(n) };
    for (i, (&p, span)) in ring.points.iter().zip(&ring.spans).enumerate() {
        let prev = ring.points[(i + n - 1) % n];
        let next = ring.points[(i + 1) % n];
        let (a, b) = (left(prev, p), left(p, next));
        let (mx, my) = (a.0 + b.0, a.1 + b.1);
        let ml = mx.hypot(my);
        // The cosine of the half angle between the bisector and each edge's normal: the miter
        // reaches 1 / cos times the inset.
        let cos = if ml < 1e-9 { 0.0 } else { (mx * a.0 + my * a.1) / ml };
        // Which way the outline turns at `p`: an inset towards `side` opens the corner when the
        // turn is against it.
        let turn = (p - prev).cross(next - p);
        let edges = (a.0 != 0.0 || a.1 != 0.0) && (b.0 != 0.0 || b.1 != 0.0);
        out.points.push(p);
        if edges && cos < 1.0 / MITER_LIMIT && turn * side < 0.0 {
            out.spans.push(None);
            out.points.push(p);
        }
        out.spans.push(*span);
    }
    out
}
