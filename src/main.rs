//! The SolidCraft adapter: answers the B-rep protocol's requests on stdin and stdout with the
//! solidcore kernel SolidCraft ships, so the conformance suite can drive that kernel.

use std::cell::RefCell;
use std::process::ExitCode;
use std::rc::Rc;

use brep_protocol::{Op, Refusal, model, model::Primitive, profile};
use solidcore::EntityRef;
use solidcore::blend::fillet_executive::api::{
    fillet_edges, fillet_edges_per_edge, variable_radius_fillet,
};
use solidcore::blend::fillet_solver::p1::FilletSurfaceGeneratorType;
use solidcore::boolean::api_brep::{boolean_difference, boolean_intersection, boolean_union};
use solidcore::curve::curve::Curve;
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::intersect::point_class::PointClassType;
use solidcore::mass::integrator::properties::brep_mass_properties;
use solidcore::mass::integrator::properties::{PropertyAttributes, brep_area, brep_volume};
use solidcore::nurbs::b_spline_curve::BSplineCurveForm;
use solidcore::offset::offset_executive::p2::{
    OffsetOptions, ShellOptions, offset_brep_simple, shell_brep_simple,
};
use solidcore::primitive::api_curves::{CanonicalCurveParams, create_canonical_curve};
use solidcore::primitive::api_curves::{create_analytic_circle, create_arc, create_line_segment};
use solidcore::primitive::api_primitives as api;
use solidcore::primitive::primitive_creation::SweepOptions;
use solidcore::query::api_queries::brep_bounding_box;
use solidcore::query::api_queries::{edge_closest_point, edge_parameter_range};
use solidcore::query::api_queries::{face_classify_uv, get_faces, surface_closest_point};
use solidcore::query::brep::BrepClassifier;
use solidcore::sweep::api_brep::create_curve_sweep;
use solidcore::sweep::api_brep::{create_linear_sweep, create_rotational_sweep};
use solidcore::sweep::api_surfaces::create_skin_primitive;
use solidcore::topology::brep::validation::{CheckLevel, check_brep};
use solidcore::{Brep, Point3, Vec3};
use solidcore::{EdgeId, FaceId};

/// https://github.com/tangency-labs/solidcraft/releases/tag/v0.47.0, the `solidcore` rev in Cargo.toml.
/// The solidcore crate's own version does not move with SolidCraft's releases, so this names them.
const VERSION: &str = "0.47.0";

/// The integrator's accuracy: relative, as solidcore's own tests use.
const ACCURACY: f64 = 1e-7;

struct Solidcore(solidcore::Kernel);

fn p3(a: model::Vec3) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

fn v3(a: model::Vec3) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn err(e: solidcore::SolidError) -> Refusal {
    Refusal::failed(e.to_string())
}

/// Moves a solid built about the world origin onto `frame`.
fn place(k: &mut solidcore::Kernel, mut b: Brep, f: &model::Frame) -> Result<Brep, Refusal> {
    let z = v3(f.x).cross(v3(f.y));
    if z.dot(v3(f.z)) < 0.0 {
        return Err("a left-handed frame".into());
    }
    let placement = Axis2Placement::new(p3(f.origin), v3(f.x), v3(f.y));
    solidcore::topology::brep::p5::transform(k, &mut b, &placement, None, false).map_err(err)?;
    Ok(b)
}

/// Points along `c` over `[a, b]`, split until each chord's middle lies within `tol` of the curve.
fn sample(
    k: &solidcore::Kernel,
    c: &solidcore::curve::curve::Curve,
    a: f64,
    b: f64,
    tol: f64,
    out: &mut Vec<[f64; 3]>,
    depth: u32,
) -> Result<(), Refusal> {
    let at = |t: f64| -> Result<Vec3, Refusal> {
        let v = c.evaluate(k, t, 0, false, false).map_err(err)?;
        Ok(v[0])
    };
    let (pa, pb, pm) = (at(a)?, at(b)?, at(0.5 * (a + b))?);
    let mid = (pa + pb) * 0.5;
    if depth >= 20 || ((pm - mid).length() <= tol && depth >= 2) {
        out.push([pa.x, pa.y, pa.z]);
        return Ok(());
    }
    sample(k, c, a, 0.5 * (a + b), tol, out, depth + 1)?;
    sample(k, c, 0.5 * (a + b), b, tol, out, depth + 1)
}

fn is_empty(b: &Brep) -> bool {
    b.face_list().is_empty()
}

/// How a solid was made, so each boolean can rebuild its operands: a boolean consumes them, and
/// `Brep::clone` keeps the original's geometry owners, which the checker rejects in the copy and in
/// anything built from it.
enum Recipe {
    Make(Primitive),
    Boolean(Op, Rc<Recipe>, Rc<Recipe>),
    /// Moved by the rows `[M | t]`, `M` a rotation or mirror times a scale.
    Move([[f64; 4]; 3], Rc<Recipe>),
    /// Edges named by points rounded each by its own radius.
    Blend {
        base: Rc<Recipe>,
        edges: Vec<model::Vec3>,
        sizes: Vec<f64>,
    },
    /// One edge rounded by a radius running evenly from `r_from` at its end nearest `from`.
    Variable {
        base: Rc<Recipe>,
        edge: model::Vec3,
        from: model::Vec3,
        r_from: f64,
        r_to: f64,
    },
    /// Grown by `distance` all round, convex edges rounded.
    Offset(Rc<Recipe>, f64),
    /// Hollowed to walls `thickness` thick, open through the faces the points lie on.
    Shell {
        base: Rc<Recipe>,
        open: Vec<model::Vec3>,
        thickness: f64,
    },
    /// The smooth solid through profile `k` in the frame's xy plane moved `heights[k]` up its z.
    LoftThrough {
        frame: model::Frame,
        profiles: Vec<profile::Profile>,
        heights: Vec<f64>,
    },
}

struct Solid {
    recipe: Rc<Recipe>,
    brep: RefCell<Brep>,
}

fn build(k: &mut solidcore::Kernel, r: &Recipe) -> Result<Brep, Refusal> {
    match r {
        Recipe::Make(p) => make(k, p),
        Recipe::Boolean(op, a, b) => {
            let (a, b) = (build(k, a)?, build(k, b)?);
            match op {
                Op::Union => boolean_union(k, a, b),
                Op::Intersection => boolean_intersection(k, a, b),
                Op::Difference => boolean_difference(k, a, b),
            }
            .map_err(err)
        }
        Recipe::Move(m, base) => {
            let mut b = build(k, base)?;
            moved(k, &mut b, m)?;
            Ok(b)
        }
        Recipe::Blend { base, edges, sizes } => {
            let mut b = build(k, base)?;
            // The named edges alone: following a tangent chain is the kernel's to do.
            let ids = edges_at(k, &b, edges)?;
            if sizes.iter().all(|&x| x == sizes[0]) {
                // Its section code for a round is 1.
                fillet_edges(k, &mut b, &ids, sizes[0], 1, 1, 1.0).map_err(err)?;
            } else {
                let kinds = vec![FilletSurfaceGeneratorType::Circular; ids.len()];
                fillet_edges_per_edge(k, &mut b, &ids, sizes, &kinds, 1, 1.0).map_err(err)?;
            }
            Ok(b)
        }
        Recipe::Variable {
            base,
            edge,
            from,
            r_from,
            r_to,
        } => {
            let mut b = build(k, base)?;
            let e = edges_at(k, &b, &[*edge])?[0];
            // Its law runs from the edge's start: which end is `from`.
            let mid = edge_parameter_range(&b, e).map_err(err)?.mid();
            let t = edge_closest_point(k, &b, e, v3(*from))
                .map_err(err)?
                .parameter;
            let (r0, r1) = if t <= mid {
                (*r_from, *r_to)
            } else {
                (*r_to, *r_from)
            };
            variable_radius_fillet(k, &mut b, &[e], r0, r1, 1, 1).map_err(err)?;
            Ok(b)
        }
        Recipe::Offset(base, distance) => {
            let mut b = build(k, base)?;
            let options = OffsetOptions {
                distance: *distance,
                extend_convex_edges: false,
                merge_results: true,
            };
            offset_brep_simple(k, &mut b, options).map_err(err)
        }
        Recipe::Shell {
            base,
            open,
            thickness,
        } => {
            let mut b = build(k, base)?;
            let faces = faces_at(k, &mut b, open)?;
            let options = ShellOptions {
                offset: OffsetOptions {
                    distance: -thickness,
                    extend_convex_edges: false,
                    merge_results: true,
                },
                create_offset_solid: false,
                merge_sheet_side_walls: false,
            };
            shell_brep_simple(k, &b, options, &faces).map_err(err)
        }
        Recipe::LoftThrough {
            frame,
            profiles,
            heights,
        } => {
            if profiles.len() < 2 || heights.windows(2).any(|w| w[1] <= w[0]) {
                return Err("a loft's heights must rise through two sections or more".into());
            }
            let sections = profiles
                .iter()
                .zip(heights)
                .map(|(pr, &h)| profile_curves(k, pr, h))
                .collect::<Result<Vec<_>, _>>()?;
            let degree = (sections.len() - 1).min(3);
            let b = skin(k, &sections, degree)?;
            place(k, b, frame)
        }
    }
}

/// The face each point lies on: the nearest whose surface passes within 1e-6 mm of the point and
/// whose trimmed area holds it there, by solidcore's closest-point and face classification.
fn faces_at(
    k: &solidcore::Kernel,
    b: &mut Brep,
    points: &[model::Vec3],
) -> Result<Vec<FaceId>, Refusal> {
    let mut found = Vec::with_capacity(points.len());
    for &p in points {
        let mut best: Option<(f64, FaceId)> = None;
        for f in get_faces(b).map_err(err)? {
            let Some(surface) = b.face(f).map_err(err)?.surface.clone() else {
                continue;
            };
            let Ok(near) = surface_closest_point(k, &surface, p3(p)) else {
                continue;
            };
            let Some(&uv) = near.uvs.first() else {
                continue;
            };
            let d = near.closest.distance;
            let held = matches!(
                face_classify_uv(k, b, f, uv),
                Ok(PointClassType::Face | PointClassType::Edge | PointClassType::Vertex)
            );
            if held && d <= 1e-6 && best.is_none_or(|(least, _)| d < least) {
                best = Some((d, f));
            }
        }
        let (_, f) =
            best.ok_or_else(|| Refusal::invalid(format!("no face passes through {p:?}")))?;
        found.push(f);
    }
    Ok(found)
}

/// The solid lofted through sections of matching curves, `degree` across them, capped.
fn skin(
    k: &mut solidcore::Kernel,
    sections: &[Vec<Curve>],
    degree: usize,
) -> Result<Brep, Refusal> {
    let counts: Vec<usize> = sections.iter().map(Vec::len).collect();
    if counts.iter().any(|&n| n != counts[0]) {
        return Err("sections of different curve counts".into());
    }
    let refs: Vec<Option<&Curve>> = sections.iter().flatten().map(Some).collect();
    Ok(create_skin_primitive(k, &counts, &refs, degree, true)
        .map_err(err)?
        .brep)
}

/// A spline's control points and knots with both ends clamped: its first and last parameters
/// inserted until each is a knot `degree` times, the same curve over the same parameters.
fn clamped(sp: &model::Spline) -> (Vec<[f64; 2]>, Vec<f64>) {
    let p = sp.degree;
    let (mut u, mut c) = (sp.knots.clone(), sp.ctrl.clone());
    let (a, b) = (sp.knots[p], sp.knots[sp.ctrl.len()]);
    for t in [a, b] {
        while u.iter().filter(|&&x| x == t).count() < p {
            insert_knot(p, &mut u, &mut c, t);
        }
    }
    let first = u.iter().rposition(|&x| x == a).unwrap_or(p) - p;
    let last = u.iter().position(|&x| x == b).unwrap_or(c.len()) - 1;
    let inner = u.iter().copied().filter(|&x| x > a && x < b);
    let knots = std::iter::repeat_n(a, p + 1)
        .chain(inner)
        .chain(std::iter::repeat_n(b, p + 1))
        .collect();
    (c[first..=last].to_vec(), knots)
}

/// Boehm's insertion of the knot `t`: one more control point, the curve unchanged.
fn insert_knot(p: usize, u: &mut Vec<f64>, c: &mut Vec<[f64; 2]>, t: f64) {
    let n = c.len();
    let mut k = p;
    while k + 1 < n && u[k + 1] <= t {
        k += 1;
    }
    let q = (0..=n)
        .map(|i| {
            if i + p <= k {
                c[i]
            } else if i > k {
                c[i - 1]
            } else {
                let a = (t - u[i]) / (u[i + p] - u[i]);
                [
                    (1.0 - a) * c[i - 1][0] + a * c[i][0],
                    (1.0 - a) * c[i - 1][1] + a * c[i][1],
                ]
            }
        })
        .collect();
    u.insert(k + 1, t);
    *c = q;
}

/// A clamped B-spline, or a rational one with `weights`, from its control points and its full
/// knot vector.
fn nurbs(
    points: &[Point3],
    knots: &[f64],
    weights: Option<&[f64]>,
    degree: usize,
) -> Result<Curve, Refusal> {
    let (mut unique, mut mults) = (Vec::new(), Vec::new());
    for &t in knots {
        if unique.last() == Some(&t) {
            *mults.last_mut().unwrap() += 1;
        } else {
            unique.push(t);
            mults.push(1);
        }
    }
    let curve = create_canonical_curve(&CanonicalCurveParams {
        points,
        knots: &unique,
        knot_multiplicities: &mults,
        weights,
        degree,
        form: BSplineCurveForm::Unspecified,
    })
    .map_err(err)?;
    Ok(Curve::from(curve))
}

/// A sweep's path from the origin along z, as one rational quadratic B-spline: each straight
/// piece a span of three points in line, each bend spans of a quarter turn or less, exact arcs.
fn path_curve(path: &[model::PathSeg]) -> Result<Curve, Refusal> {
    let (mut at, mut d, mut x) = (
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    );
    let (mut pts, mut ws) = (vec![at], vec![1.0]);
    let mut spans = 0usize;
    for seg in path {
        match *seg {
            model::PathSeg::Line { length } => {
                let end = at + d * length;
                pts.extend([at + d * (length / 2.0), end]);
                ws.extend([1.0, 1.0]);
                at = end;
                spans += 1;
            }
            model::PathSeg::Arc { radius, angle } => {
                let centre = at + x * radius;
                // The point `θ` round from the start, about the axis along y through `centre`.
                let out = |th: f64| (x * th.cos() - d * th.sin()) * -1.0;
                let n = (angle.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.0) as usize;
                let step = angle / n as f64;
                for i in 0..n {
                    let mid = step * (i as f64 + 0.5);
                    let w = (step / 2.0).cos();
                    let end = centre + out(step * (i as f64 + 1.0)) * radius;
                    pts.extend([centre + out(mid) * (radius / w), end]);
                    ws.extend([w, 1.0]);
                    spans += 1;
                }
                at = centre + out(angle) * radius;
                let (c, sn) = (angle.cos(), angle.sin());
                (d, x) = (d * c + x * sn, x * c - d * sn);
            }
        }
    }
    let knots: Vec<f64> = std::iter::repeat_n(0.0, 3)
        .chain((1..spans).flat_map(|i| [i as f64; 2]))
        .chain(std::iter::repeat_n(spans as f64, 3))
        .collect();
    nurbs(&pts, &knots, Some(&ws), 2)
}

/// The edge each point lies on: the nearest, which must pass within 1e-6 mm.
fn edges_at(
    k: &solidcore::Kernel,
    b: &Brep,
    points: &[model::Vec3],
) -> Result<Vec<EdgeId>, Refusal> {
    let all = solidcore::topology::brep::p1::edges(b, None).map_err(err)?;
    points
        .iter()
        .map(|&p| {
            all.iter()
                .filter_map(|&e| {
                    let c = edge_closest_point(k, b, e, v3(p)).ok()?;
                    Some((c.closest.distance, e))
                })
                .filter(|(d, _)| *d <= 1e-6)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, e)| e)
                .ok_or_else(|| Refusal::invalid(format!("no edge passes through {p:?}")))
        })
        .collect()
}

/// `b` moved by the rows `[M | t]`, a rotation or mirror times a scale. solidcore's one transform
/// scales about the origin along its axes, then places, so a mirror is a rotation after a scale
/// of −1 along z. A stretch or a shear is not one transform, and is refused.
fn moved(k: &mut solidcore::Kernel, b: &mut Brep, m: &[[f64; 4]; 3]) -> Result<(), Refusal> {
    let gram = |i: usize, j: usize| (0..3).map(|r| m[r][i] * m[r][j]).sum::<f64>();
    let g0 = gram(0, 0);
    let similar = (0..3)
        .all(|i| (0..3).all(|j| (gram(i, j) - if i == j { g0 } else { 0.0 }).abs() <= 1e-12 * g0));
    if !similar {
        return Err(Refusal::unsupported(
            "a stretch or a shear is not one solidcore transform",
        ));
    }
    let col = |j: usize| Vec3::new(m[0][j], m[1][j], m[2][j]);
    let det = col(0).dot(col(1).cross(col(2)));
    if det.abs() < 1e-300 {
        return Err("a degenerate motion".into());
    }
    let c = det.abs().cbrt();
    let flip = if det < 0.0 { -1.0 } else { 1.0 };
    // M = R·diag(c, c, flip·c), so R's columns are M's, the third flipped, over c.
    let (x, y) = (col(0) * (1.0 / c), col(1) * (1.0 / c));
    let placement = Axis2Placement::new(Point3::new(m[0][3], m[1][3], m[2][3]), x, y);
    solidcore::topology::brep::p5::transform(
        k,
        b,
        &placement,
        Some(Vec3::new(c, c, flip * c)),
        false,
    )
    .map_err(err)
}

/// The suite's profile loops as solidcore curves in the plane `z` up the xy plane.
fn curves(p: &profile::Profile, z: f64) -> Result<Vec<Curve>, Refusal> {
    let mut out = Vec::new();
    for lp in &p.loops {
        for s in lp {
            match *s {
                profile::Seg::Line { a, b } => {
                    let line =
                        create_line_segment(Point3::new(a[0], a[1], z), Point3::new(b[0], b[1], z))
                            .map_err(err)?;
                    out.push(Curve::from(line));
                }
                profile::Seg::Arc {
                    center,
                    radius,
                    start,
                    sweep,
                } => {
                    let c = Point3::new(center[0], center[1], z);
                    if sweep.abs() >= std::f64::consts::TAU - 1e-12 {
                        return Err("a whole circle needs the kernel".into());
                    }
                    let (a, b) = if sweep > 0.0 {
                        (start, start + sweep)
                    } else {
                        (start + sweep, start)
                    };
                    let arc = create_arc(c, radius, a.to_degrees(), b.to_degrees()).map_err(err)?;
                    out.push(Curve::from(arc));
                }
                profile::Seg::Spline(ref sp) => {
                    let (ctrl, knots) = clamped(sp);
                    let pts: Vec<Point3> =
                        ctrl.iter().map(|q| Point3::new(q[0], q[1], z)).collect();
                    out.push(nurbs(&pts, &knots, None, sp.degree)?);
                }
                profile::Seg::Conic { a, apex, b, w } => {
                    let pts: Vec<Point3> = [a, apex, b]
                        .iter()
                        .map(|q| Point3::new(q[0], q[1], z))
                        .collect();
                    let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
                    out.push(nurbs(&pts, &knots, Some(&[1.0, w, 1.0]), 2)?);
                }
                profile::Seg::Gn { .. } => {
                    return Err("a blend-section segment".into());
                }
                // A kind of segment added to the protocol after this adapter was written.
                _ => {
                    return Err(Refusal::unsupported(
                        "this kind of profile segment is not supported",
                    ));
                }
            }
        }
    }
    Ok(out)
}

/// Whole circles, made by the kernel as analytic circles, in the plane `z` up the xy plane.
fn circles(k: &solidcore::Kernel, p: &profile::Profile, z: f64) -> Result<Vec<Curve>, Refusal> {
    let mut out = Vec::new();
    for lp in &p.loops {
        for s in lp {
            if let profile::Seg::Arc {
                center,
                radius,
                sweep,
                ..
            } = *s
                && sweep.abs() >= std::f64::consts::TAU - 1e-12
            {
                let c = Point3::new(center[0], center[1], z);
                out.push(Curve::from(
                    create_analytic_circle(k, c, radius).map_err(err)?,
                ));
            }
        }
    }
    Ok(out)
}

/// A profile's curves `z` up the xy plane: its whole circles as analytic circles, the rest as
/// segments.
fn profile_curves(
    k: &solidcore::Kernel,
    p: &profile::Profile,
    z: f64,
) -> Result<Vec<Curve>, Refusal> {
    let mut whole = circles(k, p, z)?;
    let rest = profile::Profile {
        loops: p
            .loops
            .iter()
            .map(|lp| {
                lp.iter()
                    .filter(|s| {
                        !matches!(s, profile::Seg::Arc { sweep, .. }
                            if sweep.abs() >= std::f64::consts::TAU - 1e-12)
                    })
                    .cloned()
                    .collect()
            })
            .collect(),
    };
    whole.extend(curves(&rest, z)?);
    Ok(whole)
}

fn make(k: &mut solidcore::Kernel, p: &Primitive) -> Result<Brep, Refusal> {
    let o = Point3::new(0.0, 0.0, 0.0);
    let (b, frame) = match p {
        Primitive::Box { frame, size } => (api::create_box(k, o, size[0], size[1], size[2]), frame),
        Primitive::Cylinder {
            frame,
            radius,
            height,
        } => (api::create_cylinder(k, o, *radius, *height), frame),
        Primitive::Sphere { frame, radius } => (api::create_sphere(k, o, *radius), frame),
        Primitive::Torus {
            frame,
            major,
            minor,
        } => (api::create_torus(k, o, *major, *minor), frame),
        Primitive::Cone {
            frame,
            bottom,
            top,
            height,
        } => (api::create_cone(k, o, *bottom, *top, *height), frame),
        Primitive::Extrude {
            frame,
            profile,
            height,
        } => {
            let cs = profile_curves(k, profile, 0.0)?;
            let refs: Vec<Option<&Curve>> = cs.iter().map(Some).collect();
            let b = create_linear_sweep(k, &refs, Vec3::new(0.0, 0.0, 1.0), *height, true);
            (b, frame)
        }
        Primitive::Revolve {
            frame,
            profile,
            angle,
        } => {
            // Drawn with x out from the axis and y along it, turned about y; placed so that
            // local y is the frame's z and the turn runs from its x toward its y.
            let cs = profile_curves(k, profile, 0.0)?;
            let refs: Vec<Option<&Curve>> = cs.iter().map(Some).collect();
            let whole = *angle >= std::f64::consts::TAU - 1e-12;
            let mut b = create_rotational_sweep(
                k,
                &refs,
                o,
                Vec3::new(0.0, 1.0, 0.0),
                angle.to_degrees(),
                !whole,
            )
            .map_err(err)?;
            let placement = Axis2Placement::new(p3(frame.origin), v3(frame.x), v3(frame.z));
            solidcore::topology::brep::p5::transform(k, &mut b, &placement, None, false)
                .map_err(err)?;
            return Ok(b);
        }
        Primitive::Loft {
            frame,
            profile,
            height,
            top,
            ..
        } => {
            let sections = [
                profile_curves(k, profile, 0.0)?,
                profile_curves(k, top, *height)?,
            ];
            let b = skin(k, &sections, 1)?;
            return place(k, b, frame);
        }
        Primitive::Sweep {
            frame,
            profile,
            path,
        } => {
            let cs = profile_curves(k, profile, 0.0)?;
            let refs: Vec<Option<&Curve>> = cs.iter().map(Some).collect();
            let along = path_curve(path)?;
            let options = SweepOptions {
                cap_ends: true,
                profile_at_path_start: true,
                ..SweepOptions::default()
            };
            let swept = create_curve_sweep(k, &refs, Some(&along), None, None, Some(&options))
                .map_err(err)?;
            return place(k, swept.brep, frame);
        }
        _ => return Err(Refusal::unsupported("this constructor is not supported")),
    };
    place(k, b.map_err(err)?, frame)
}

impl Solidcore {
    fn solid(&mut self, recipe: Recipe) -> Result<Solid, Refusal> {
        let k = &mut self.0;
        let brep = build(k, &recipe)?;
        Ok(Solid {
            recipe: Rc::new(recipe),
            brep: RefCell::new(brep),
        })
    }

    fn boolean_of(&mut self, op: Op, a: &Solid, b: &Solid) -> Result<Solid, Refusal> {
        self.solid(Recipe::Boolean(op, a.recipe.clone(), b.recipe.clone()))
    }

    fn blend(
        &mut self,
        s: &Solid,
        edges: &[model::Vec3],
        sizes: Vec<f64>,
    ) -> Result<Solid, Refusal> {
        if edges.is_empty() || sizes.len() != edges.len() {
            return Err("a blend names an edge and a size for each".into());
        }
        self.solid(Recipe::Blend {
            base: s.recipe.clone(),
            edges: edges.to_vec(),
            sizes,
        })
    }
}

/// Mass properties at unit density, the moments about `origin`.
fn mass(
    k: &solidcore::Kernel,
    b: &mut Brep,
    origin: Point3,
) -> Result<solidcore::mass::integrator::properties::MassProperties, Refusal> {
    let store = AttributeStore::new();
    let key_of = |_| ObjectKey(0);
    let attrs = PropertyAttributes {
        store: &store,
        key_of: &key_of,
    };
    brep_mass_properties(k, b, ACCURACY, 1.0, origin, &attrs).map_err(err)
}

impl brep_protocol::Kernel for Solidcore {
    type Solid = Solid;

    fn name(&self) -> String {
        "solidcraft".into()
    }

    fn version(&self) -> String {
        VERSION.into()
    }

    fn tolerance(&self) -> f64 {
        1e-5
    }

    fn configuration(&self) -> String {
        "zone tolerance set to 1e-7 mm; bodies built at the default 1e-5 mm".into()
    }

    fn make(&mut self, p: &Primitive) -> Result<Solid, Refusal> {
        self.solid(Recipe::Make(p.clone()))
    }

    fn union(&mut self, a: &Solid, b: &Solid) -> Result<Solid, Refusal> {
        self.boolean_of(Op::Union, a, b)
    }

    fn intersection(&mut self, a: &Solid, b: &Solid) -> Result<Solid, Refusal> {
        self.boolean_of(Op::Intersection, a, b)
    }

    fn difference(&mut self, a: &Solid, b: &Solid) -> Result<Solid, Refusal> {
        self.boolean_of(Op::Difference, a, b)
    }

    fn volume(&mut self, s: &Solid) -> Result<f64, Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok(0.0);
        }
        let k = &self.0;
        let store = AttributeStore::new();
        let key_of = |_| ObjectKey(0);
        let attrs = PropertyAttributes {
            store: &store,
            key_of: &key_of,
        };
        brep_volume(k, &mut s.brep.borrow_mut(), ACCURACY, &attrs).map_err(err)
    }

    fn area(&mut self, s: &Solid) -> Result<f64, Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok(0.0);
        }
        let k = &self.0;
        brep_area(k, &mut s.brep.borrow_mut(), ACCURACY).map_err(err)
    }

    fn centroid(&mut self, s: &Solid) -> Result<Option<model::Vec3>, Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok(None);
        }
        let k = &self.0;
        let c = mass(k, &mut s.brep.borrow_mut(), Point3::new(0.0, 0.0, 0.0))?.centroid;
        Ok(Some([c.x, c.y, c.z]))
    }

    fn inertia(&mut self, s: &Solid) -> Result<[[f64; 3]; 3], Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok([[0.0; 3]; 3]);
        }
        let k = &self.0;
        let b = &mut s.brep.borrow_mut();
        let c = mass(k, b, Point3::new(0.0, 0.0, 0.0))?.centroid;
        // About the centroid: axial moments, and the raw products yz, zx and xy.
        let m = mass(k, b, c)?;
        let (i, p) = (m.inertia, m.products);
        Ok([[i.x, -p.z, -p.y], [-p.z, i.y, -p.x], [-p.y, -p.x, i.z]])
    }

    fn transform(&mut self, s: &Solid, matrix: [[f64; 4]; 3]) -> Result<Solid, Refusal> {
        self.solid(Recipe::Move(matrix, s.recipe.clone()))
    }

    fn fillet(&mut self, s: &Solid, edges: &[model::Vec3], radius: f64) -> Result<Solid, Refusal> {
        self.blend(s, edges, vec![radius; edges.len()])
    }

    fn fillet_radii(
        &mut self,
        s: &Solid,
        edges: &[model::Vec3],
        radii: &[f64],
    ) -> Result<Solid, Refusal> {
        self.blend(s, edges, radii.to_vec())
    }

    fn fillet_variable(
        &mut self,
        s: &Solid,
        edge: model::Vec3,
        from: model::Vec3,
        r_from: f64,
        r_to: f64,
    ) -> Result<Solid, Refusal> {
        self.solid(Recipe::Variable {
            base: s.recipe.clone(),
            edge,
            from,
            r_from,
            r_to,
        })
    }

    fn offset(&mut self, s: &Solid, distance: f64) -> Result<Solid, Refusal> {
        self.solid(Recipe::Offset(s.recipe.clone(), distance))
    }

    fn shell(&mut self, s: &Solid, open: &[model::Vec3], thickness: f64) -> Result<Solid, Refusal> {
        self.solid(Recipe::Shell {
            base: s.recipe.clone(),
            open: open.to_vec(),
            thickness,
        })
    }

    fn loft_through(
        &mut self,
        frame: &model::Frame,
        profiles: &[profile::Profile],
        heights: &[f64],
    ) -> Result<Solid, Refusal> {
        self.solid(Recipe::LoftThrough {
            frame: *frame,
            profiles: profiles.to_vec(),
            heights: heights.to_vec(),
        })
    }

    fn classify(&mut self, s: &Solid, points: &[model::Vec3]) -> Result<Vec<model::Side>, Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok(vec![model::Side::Out; points.len()]);
        }
        let k = &self.0;
        let b = s.brep.borrow();
        let mut c = BrepClassifier::of_brep(&b);
        points
            .iter()
            .map(|&p| {
                let pc = c.point_3d_classify(k, p3(p), 1e-7, false).map_err(err)?;
                // A point in a region is in when the region is material; any other class the
                // kernel gives is a place on the boundary: a face, an edge or a vertex.
                Ok(match (pc.point_class(), pc.object()) {
                    (PointClassType::Region, Some(EntityRef::Region(r))) => {
                        if c.brep().region(r).map_err(err)?.is_void == 0 {
                            model::Side::In
                        } else {
                            model::Side::Out
                        }
                    }
                    (PointClassType::Region, _) => model::Side::Out,
                    (PointClassType::Unknown, _) => {
                        return Err(Refusal::failed(format!(
                            "the classifier could not place {p:?}"
                        )));
                    }
                    _ => model::Side::On,
                })
            })
            .collect()
    }

    fn bounding_box(&mut self, s: &Solid) -> Result<[model::Vec3; 2], Refusal> {
        let k = &mut self.0;
        let b = brep_bounding_box(k, &s.brep.borrow(), true).map_err(err)?;
        let (lo, hi) = (b.min(), b.max());
        Ok([[lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z]])
    }

    fn section(
        &mut self,
        s: &Solid,
        origin: model::Vec3,
        normal: model::Vec3,
        tolerance: f64,
    ) -> Result<Vec<Vec<model::Vec3>>, Refusal> {
        use solidcore::topology::brep::planar_section::{
            PlanarSectionOptions, SectionOutputs, create_planar_section_curves,
        };
        let k = &mut self.0;
        let opts = PlanarSectionOptions {
            outputs: SectionOutputs {
                curves_3d: true,
                ..Default::default()
            },
            ..Default::default()
        };
        let n = v3(normal) * (1.0 / v3(normal).length());
        let cut = create_planar_section_curves(k, &mut s.brep.borrow_mut(), p3(origin), n, &opts)
            .map_err(err)?;
        let mut pieces = Vec::new();
        for c in cut.curves_3d.unwrap_or_default() {
            let range = c.natural_interval();
            let mut pts = Vec::new();
            sample(k, &c, range.min(), range.max(), tolerance, &mut pts, 0)?;
            let end = c.evaluate(k, range.max(), 0, false, false).map_err(err)?[0];
            pts.push([end.x, end.y, end.z]);
            pieces.push(pts);
        }
        Ok(pieces)
    }

    fn check(&mut self, s: &Solid) -> Result<Vec<String>, Refusal> {
        if is_empty(&s.brep.borrow()) {
            return Ok(Vec::new());
        }
        // The checker repairs what it checks (reversed curves, control nets that turn back), so
        // it runs on a fresh build and leaves the solid the other answers measure as it was.
        let k = &mut self.0;
        let mut fresh = build(k, &s.recipe)?;
        Ok(check_brep(k, &mut fresh, CheckLevel::Level2)
            .map_err(err)?
            .lines())
    }

    // The functions of the protocol that solidcore has one call for. The others are left out:
    //
    // - Operations the solidcore crate did not have at 1.0.1, STEP and meshing among them (not yet
    //   reviewed against SolidCraft v0.47.0):
    //   fillet_g2, fillet_conic, fillet_setback, fillet_setback_g2, fillet_faces, full_round,
    //   curvature, project, slab, offset_faces, chamfer_two, write_step, read_step, mesh, display,
    //   draft, delete_faces, move_faces, thicken, sew, sweep_path and interference.
    // - Operations solidcore reaches in its own way, where the protocol asks for one call of
    //   another shape. The adapter does not assemble them, so that what is scored is solidcore.
    //   - screw: it sweeps a profile along a path or straight across; the protocol's screw turns
    //     the profile about an axis as it rises.
    //   - split: its plane cut keeps one side as an open sheet; the protocol asks for both sides
    //     as solids.
    //   - section_area: its planar section gives curves; the protocol asks for the area they
    //     bound.
    //   - raycast: its ray fire gives the nearest crossing; the protocol asks for every one.
    //   - betti: its census counts voids; the protocol also asks for pieces and tunnels.
    //   - normal: it gives a surface's normal; the protocol asks for the solid's outward one.
    //   - affine: its transform scales along its own axes; the protocol's affine map may shear.
    //   - chamfer and chamfer_distances: it sizes a chamfer by a rolling ball; the protocol sizes
    //     one by its setback.
    //   - union_all and difference_all: its boolean of two lists gives a B-rep for each separate
    //     solid; the protocol's boolean of many gives one solid. Mapping one onto the other is not
    //     worked out here.
    //   - distance and closest: its distance and closest-point queries measure to a body's
    //     boundary; the protocol's measure to the solid, and are zero inside it.
    fn supports(&self) -> Vec<String> {
        [
            "box",
            "cylinder",
            "sphere",
            "torus",
            "cone",
            "extrude",
            "revolve",
            "loft",
            "loft_through",
            "sweep",
            "union",
            "intersection",
            "difference",
            "fillet",
            "fillet_radii",
            "fillet_variable",
            "offset",
            "shell",
            "volume",
            "area",
            "classify",
            "check",
            "centroid",
            "inertia",
            "section",
            "bounding_box",
            "transform",
        ]
        .map(String::from)
        .to_vec()
    }
}

fn main() -> ExitCode {
    // The suite asks 1e-7 mm; solidcore's default zone is 1e-5, the rest derived from it as its
    // defaults derive them.
    let zone = 1e-7;
    let options = solidcore::KernelOptions {
        tolerances: solidcore::Tolerances {
            zone_tol_3d: zone,
            xsect_tol_3d: 2.0 * zone,
            approx_tol_3d: zone / 2.0,
            ..solidcore::Tolerances::default()
        },
        ..solidcore::KernelOptions::default()
    };
    let mut kernel = Solidcore(solidcore::Kernel::new(&options));
    match brep_protocol::serve(&mut kernel) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
