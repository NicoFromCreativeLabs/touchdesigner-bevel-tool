# TouchDesigner Bevel Tool

TouchDesigner has no native Bevel SOP. This is a Script SOP (plus a packaged `.tox` component) that replicates the two bevel modes you'd expect from Blender/Houdini: **Edge Bevel** and **Vertex Bevel**.

## Download

- **`Bevel.tox`** — the packaged, ready-to-use component. Drag it into any TouchDesigner network, or drop it into your palette folder (`Documents/Derivative/Palette/`) to have it show up in TD's own Palette browser.
- **`bevel_script_sop.py`** — the underlying Script SOP source, if you want to inspect it, modify it, or build it yourself from a bare Script SOP.

## What it does

- **Edge mode** (`Mode = edge`): every edge of the input geometry is beveled with a true rounded profile, not just a single flat chamfer facet. Segment count is adjustable, corners where multiple beveled edges meet are properly capped (a correct barycentric triangular-lattice fill, with real all-quad topology once segment count allows it, not an invented center point).
- **Vertex mode** (`Mode = vertex`): truncates only selected corner vertices into a small flat cap. Edges stay perfectly sharp along their whole length right up to the corner — matching Blender's Vertex Bevel behavior, not Edge Bevel.
- Automatic point-welding before beveling, since TouchDesigner's primitive SOPs (Box, etc.) don't share points between faces by default — without this, edge-adjacency detection can't find which faces share an edge.
- Generated UVs on every piece of geometry the tool creates (local planar projection on faces, length/width-continuous on the rounded strips, exact barycentric coordinates on the corner caps).
- A **Clamp Overlap** safety clamp, so a large width value can't make the bevel self-intersect on short edges.

## Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `Width` | Float | 0.08 | Bevel width, in world units |
| `Segments` | Int | 2 | Profile segments. 1 = flat chamfer. Even values give cleaner corner topology. |
| `Mode` | Menu | `edge` | `edge` or `vertex` |
| `Weldtolerance` | Float | 0.0001 | Distance under which coincident points are treated as shared |
| `Profileshape` | Float | 0.5 | 0.25 = flat chamfer, 0.5 = circular, >0.5 = convex/square, <0.25 = concave. Approximates Blender's superellipse profile shape. |
| `Clampoverlap` | Toggle | On | Auto-shrinks width so the bevel can't self-intersect on short edges |
| `Vertexindices` | String | *(blank)* | **Vertex mode only** — comma-separated point indices to bevel. Blank = all points. |

## Known limitations

- Works correctly on manifold geometry where edges are shared by exactly two faces. Open/boundary edges are left unbeveled.
- Corner capping in Edge mode assumes exactly 3 faces meet at each vertex (true for any box-like primitive). Higher-valence vertices fall back to a simpler fan cap.
- UVs are not a seamless global unwrap — each face/strip/cap is its own local projection, correct and non-degenerate, but not continuous across the whole mesh.
- `Profileshape` is an approximation of Blender's superellipse profile curve, not an exact evaluation of that formula.

## Usage

1. Feed any closed polygonal mesh into the Script SOP's input.
2. Chain a Facet SOP after it with **Compute Normals** and **Unique Points** both on, so the new bevel geometry shades correctly.
3. Adjust `Width` / `Segments` / `Profileshape` to taste.

Shared for free use — no license terms attached beyond that. Use it, modify it, build on it.
