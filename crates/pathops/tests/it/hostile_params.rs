//! Absurd but finite or non-finite numeric parameters: no panic, hang or runaway allocation.

use vectorcraft_geom::{FillRule, PathData, Point, SubPath};
use vectorcraft_pathops::*;

const HOSTILE: [f64; 9] = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -0.0, 1e-300, 1e300, -1e300, f64::MAX];

fn square() -> PathData {
    PathData::single(SubPath::polyline(&[Point::new(0.0, 0.0), Point::new(10.0, 0.0), Point::new(10.0, 10.0), Point::new(0.0, 10.0)], true))
}

fn zigzag() -> PathData {
    let pts: Vec<Point> = (0..40).map(|i| Point::new(i as f64 * 3.0, if i % 2 == 0 { 0.0 } else { 7.0 })).collect();
    PathData::single(SubPath::polyline(&pts, false))
}

#[test]
fn edit_functions_survive_hostile_numbers() {
    for paths in [square(), zigzag()] {
        for &v in &HOSTILE {
            let _ = simplify(&paths, v);
            let _ = simplify_with(&paths, &SimplifyOptions { tolerance: v, corner_angle_deg: v, straight_lines: false, ..Default::default() });
            let _ = simplify_with(&paths, &SimplifyOptions { tolerance: 0.5, corner_angle_deg: v, straight_lines: true, ..Default::default() });
            let _ = smooth(&paths, v);
            let _ = smooth_level(&paths, v);
            let _ = remove_redundant_points(&paths, v);
            let _ = join(&[paths.clone(), zigzag()], v);
            let _ = simplify_level_tolerance(v);
            let _ = auto_simplify_level(v);
        }
    }
}

/// A stroke width of 1e200 or more with round joins used to panic inside the sweep (`Left == Ish`).
#[test]
fn stroke_and_offset_survive_hostile_numbers() {
    for paths in [square(), zigzag()] {
        for &w in &HOSTILE {
            for &m in &[f64::NAN, 0.0, 1.0, 1e300] {
                for join in [Join::Miter, Join::Round, Join::Bevel] {
                    for cap in [Cap::Butt, Cap::Round] {
                        let _ = outline_stroke(&paths, w, cap, join, m);
                    }
                    let _ = offset_path(&paths, w, join, m);
                }
            }
        }
    }
}

/// Shapes far from the origin pushed the outline de-duplication grid cell index to `i64::MAX`,
/// and the neighbour lookup `cx + 1` overflowed.
#[test]
fn pathfinder_outline_survives_far_away_shapes() {
    for &x in &[1e30, 1e300, f64::MAX / 4.0] {
        let sq = |dx: f64| {
            let p = PathData::single(SubPath::polyline(
                &[Point::new(x + dx, x), Point::new(x + dx + 10.0, x), Point::new(x + dx + 10.0, x + 10.0), Point::new(x + dx, x + 10.0)],
                true,
            ));
            Shape::new(p, FillRule::NonZero, 1)
        };
        let _ = pathfinder(PathfinderOp::Outline, &[sq(0.0), sq(5.0)]);
    }
}
