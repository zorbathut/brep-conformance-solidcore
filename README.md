# brep-conformance-solidcraft

The [brep-conformance](https://github.com/springcraft-ai/brep-conformance) adapter for
[SolidCraft](https://github.com/tangency-labs/solidcraft)'s kernel,
[solidcore](https://github.com/tangency-labs/solidcore): it puts solidcore behind the suite's
protocol, so that the suite can score it.

The suite lists it as `solidcraft`, at SolidCraft's release. solidcore is developed inside
SolidCraft's repository, in its `solidcore/` folder, and mirrored to its own repository from there.
The solidcore crate's version tracks the upstream release it was ported from, not the kernel's own
changes, so it stays at 1.0.1 while the kernel moves. A SolidCraft release names one state of the
kernel, and the adapter pins that release's commit: v0.47.0 now.

## Who maintains this

brep-conformance scores a B-rep kernel through an adapter: a small program that answers the suite's
requests with the kernel's own calls. Springcraft, which maintains the suite, wrote this adapter and
maintains it.

- **It is not SolidCraft's.** SolidCraft's maintainers did not write it or choose the tests it runs.
- **A failure can be the adapter's.** A case solidcore fails in the results may be this adapter
  driving it wrongly. Corrections are welcome: open an issue or a pull request.
- **It adds nothing to the kernel.** The adapter is pass-through, as the suite's
  [contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
  asks: one call of the kernel for each function, and anything the kernel has no one call for is
  listed as unsupported. What is scored is solidcore, not this code.
- **SolidCraft's maintainers are welcome here.** They know best how to drive solidcore, so their
  review and corrections carry the most weight. Ask in an issue to be added as a reviewer.
- **The suite decides what is scored.** An official run uses only a commit of this adapter that the
  suite's maintainers have reviewed as pass-through, whoever wrote it.

## Overview

[`src/main.rs`](src/main.rs) answers each function with solidcore's one call for it, as
[the contract](https://github.com/springcraft-ai/brep-conformance/blob/main/doc/contract.md#what-an-adapter-may-do)
asks, and lists the rest as unsupported. It builds the suite's box, cylinder, ball, ring and frustum
with solidcore's primitives; its extrusions and revolutions as linear and rotational sweeps of
curves, splines clamped by knot insertion first; its sweeps along the path as one rational quadratic
B-spline; and its lofts as skins, ruled or through every section. Each is built at the origin and
moved onto its frame. It answers the booleans, volume, area, mass properties and validity check with
solidcore's own, and classifies points with its B-rep classifier; fillets, one radius or one per
edge, and variable fillets with its fillet executive, on the edges the points name and no others;
offsets and shells with its offset executive; and any similarity with its transform, a mirror as a
scale of −1. The bounding box is solidcore's tight box. A plane section is solidcore's planar
section curves, each sampled until every chord's middle lies within the tolerance, returned as the
pieces they are.

This adapter began as
[brep-conformance-solidcore](https://github.com/springcraft-ai/brep-conformance-solidcore), which
pinned the solidcore 1.0.1 crate from crates.io. Two behaviours of solidcore showed in the results
at 1.0.1, as its source read at that release. They have not been checked again at v0.47.0, and the
adapter still answers as it did. An
offset, a shell or a mirror names its new curves under the edit's own identities while the body
still declares the Notify scheme its checker reads, so the checker rejects every such body, though
volume and area are right and each edge meets its vertices. And the zone tolerance is the constant
`1e-5`, which its source names `SM_ZONE_TOL_3D`, not the context's value: every body starts at that
tolerance whatever the kernel options say, and `set_tolerance` only raises entity tolerances. Its
checker also repairs what it checks, so the adapter checks a fresh build and leaves the solid it
measures as it was.

Listed as unsupported, since solidcore 1.0.1 had no one call of the protocol's shape for it. At
v0.47.0 the suite finds the same cases unsupported, but whether the kernel now has one call for any
of them has not been reviewed. Where
solidcore reaches the same end another way, the adapter does not assemble it, so that what is scored
is solidcore:

- The distance between two solids, and the nearest point of a solid: its queries measure to a body's
  boundary, where the protocol's measure to the solid and are zero inside it.
- Chamfers by a setback: it sizes a chamfer by a rolling ball.
- Stretches and shears: its transform scales along its own axes.
- Every crossing of a ray: its ray fire gives the nearest.
- Section areas, Betti numbers and outward normals: it gives section curves, a count of voids and a
  surface's normal.
- Splits: its plane cut keeps one side as an open sheet.
- The booleans of many: its boolean of two lists gives a B-rep for each separate solid, and mapping
  that onto the protocol's one solid is not worked out here.
- Screws: it sweeps along a path or straight across, and has a helix curve to sweep along.
- The interference of a pair, sweeps along a spline, face deletes, moves, offsets and thickens,
  two-distance chamfers, drafts, slabs, sewing, meshes and STEP, which the solidcore crate did not
  offer at 1.0.1. SolidCraft's repository has a STEP translator and a mesher in crates of their own
  beside the kernel's, which this adapter does not link.

A boolean consumes its operands, and `Brep::clone` keeps the original's geometry owners, which
solidcore's checker rejects in the copy and in anything built from it. So the adapter keeps how each
solid was made and rebuilds fresh operands for every boolean. Volume and area are integrated at an
accuracy of 1e-7: at 1e-9 the integrator fails on a ball. The kernel runs at the suite's tolerance
of 1e-7 mm, set through its options, though its bodies keep the 1e-5 mm noted above.

## Setup

The Rust toolchain [`rust-toolchain.toml`](rust-toolchain.toml) pins.

## Usage

The suite runs the adapter as a separate process. Build the adapter here, then run the suite from a
checkout of [brep-conformance](https://github.com/springcraft-ai/brep-conformance) beside this one:

```bash
cargo build --release
cd ../brep-conformance
cargo build --release -p brep-conformance
target/release/brep-conformance --adapter ../brep-conformance-solidcraft/target/release/solidcraft-adapter \
    --out solidcraft.json
```

## Licences

The adapter in this repository is under either the MIT licence
([LICENSE-MIT](LICENSE-MIT)) or the Apache License, Version 2.0
([LICENSE-APACHE](LICENSE-APACHE)), at your option.

Either licence covers the adapter and nothing else. It does not cover the kernel, and nothing here
relicenses the kernel.

solidcore is a separate work under its own licence. Its crate in SolidCraft's repository declares
`Apache-2.0`; that repository is the authority on that, not this page.

Building the adapter downloads SolidCraft's repository at the pinned commit and links its solidcore
crate in, so a built adapter contains solidcore and is subject to solidcore's terms as well as this
repository's.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).
