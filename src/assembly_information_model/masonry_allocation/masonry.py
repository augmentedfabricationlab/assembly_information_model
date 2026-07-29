"""
masonry.py
==========

A simple stretcher bond wall as a container of *slots* -- positions and
orientations where a brick can be placed -- built as an
assembly_information_model `Assembly`.

This module only builds the container: an Assembly whose parts are positioned
copies of a given template brick `Part` (typically a `CellularizedPart`),
split into a "good" half and a "bad" half. It does NOT decide which *stock*
brick ends up in which slot -- that's allocate.py's job: it creates its own
transformed copies of graded stock bricks and assigns them as each slot's
"occupant", independently of the template placeholder sitting there.

Stretcher bond convention used here:
- Each course (row) is offset by half a brick length from the course below.
- Bricks in a course are spaced by `joint` along the running direction.
- Courses are stacked along world Z by (height + joint).
- Layers (wythes) are stacked along the running direction's in-plane
  perpendicular by (width + joint); each layer repeats the same course/
  position/bond pattern, just offset sideways -- no header bond between them.
- Each course is split into two halves: the first half (by brick count) is
  the "bad" half (damaged faces oriented outward), the rest is the "good"
  half.
- "Outward" is assumed to be the -Y face of each brick (the wall's front
  face) in the straight (no `curve`) case; with a `curve`, it's the local
  -yaxis of the frame built from the curve's tangent (see below).
- Each slot copy's `frame` attribute is set directly to its slot position --
  its `mesh`/`shape` geometry is left in `part`'s own local coordinates, not
  transformed into place. Visualization/export is expected to place each
  part's geometry using its `frame` itself.

Without a `curve`, `num_bricks_per_course` is taken literally (must be even,
so the wall splits into two equal halves) and bricks run along world X.

With a `curve`, `num_bricks_per_course` is ignored: starting from the curve's
start point, whole bricks + `joint` gaps are placed one after another until
the next one would run past the curve's end -- there's no partial brick, and
the leftover length at the end is simply unused. The curve is assumed planar
and roughly horizontal (a wall footprint seen from above); each brick's
running direction follows the curve's local tangent, and layers offset along
the in-plane perpendicular to that tangent (`cross(world Z, tangent)`), not
along the curve's own (possibly out-of-plane-tilting) Frenet normal -- so the
wall stays vertical even if the curve's normal would otherwise twist. Any
compas curve is accepted; non-`Polyline` curves are discretized once via
`curve.to_polyline()` and all placement math (which needs exact,
length-uniform ``point_at``/``tangent_at``) runs on that.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame, Point, Polyline
from compas.geometry import add_vectors
from compas.geometry import scale_vector
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector

from ..assembly import Assembly


def _straight_course(stagger, L, joint, num_bricks_per_course):
    """(point, xaxis, yaxis) for each brick in one course, running along world X."""
    course = []
    for position in range(num_bricks_per_course):
        x = stagger + position * (L + joint) + L / 2.0
        course.append((Point(x, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
    return course


def _curve_course(curve, curve_length, stagger, L, joint):
    """(point, xaxis, yaxis) for each brick in one course that fits along `curve`.

    Stops as soon as the next brick would run past the curve's end -- no
    partial bricks, leftover length at the end is left unused.
    """
    course = []
    position = 0
    while True:
        s = stagger + position * (L + joint) + L / 2.0
        if s + L / 2.0 > curve_length:
            break
        t = s / curve_length
        point = curve.point_at(t)
        xaxis = normalize_vector(curve.tangent_at(t))
        yaxis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], xaxis))
        course.append((point, xaxis, yaxis))
        position += 1
    return course


def build_stretcher_bond_wall(
    part,
    joint=0.01,
    num_courses=6,
    num_bricks_per_course=10,
    num_layers=1,
    curve=None,
    outward_axis="y",
    outward_side="-",
):
    """Build an empty stretcher bond wall as an Assembly of positioned brick copies.

    Parameters
    ----------
    part : :class:`assembly_information_model.Part`
        Template brick (typically a `CellularizedPart`) copied and
        repositioned into every slot. Its own bounding geometry (via
        ``part.mesh.aabb()``) supplies the (length, width, height) used for
        slot spacing -- there's no separate `brick_size` to keep in sync.
    joint : float
        Mortar/gap thickness between bricks, in meters.
    num_courses : int
        Number of rows stacked in Z.
    num_bricks_per_course : int
        Number of brick slots per row. Must be even so the wall splits
        evenly into a "good" and a "bad" half. Ignored if `curve` is given --
        see the module docstring.
    num_layers : int, optional
        Number of parallel wythes, stacked along the in-plane perpendicular
        to the running direction by (width + joint). Default 1 (single layer).
    curve : :class:`compas.geometry.Curve` | :class:`compas.geometry.Polyline`, optional
        If given, courses follow this curve (from its start point) instead of
        a straight line along world X -- see the module docstring for the
        fitting/orientation rules.
    outward_axis, outward_side : str
        Which local face of each brick is considered "outward" / visible
        on the wall's display side (used later by allocate.py to score
        candidate bricks). Default: the brick's local -Y face.

    Returns
    -------
    :class:`assembly_information_model.Assembly`
        The wall container. Each part is a copy of `part`, repositioned to
        its slot, with these attributes set:
        - "layer": int, wythe index (0 = first)
        - "course": int, row index (0 = bottom)
        - "position": int, column index within the course
        - "half": "bad" | "good"
        - "outward_axis", "outward_side": str, str (copied from arguments)
        - "occupant": None, filled in later by allocate.py
    """
    if curve is None and num_bricks_per_course % 2 != 0:
        raise ValueError("num_bricks_per_course must be even to split into two equal halves.")

    box = part.mesh.aabb()
    L, W, H = box.xsize, box.ysize, box.zsize

    curve_length = None
    if curve is not None:
        if not isinstance(curve, Polyline):
            curve = curve.to_polyline()
        curve_length = curve.length

    wall = Assembly(name="stretcher_bond_wall")
    wall.attributes["brick_size"] = (L, W, H)
    wall.attributes["joint"] = joint
    wall.attributes["num_courses"] = num_courses
    wall.attributes["num_layers"] = num_layers
    if curve is None:
        wall.attributes["num_bricks_per_course"] = num_bricks_per_course
    else:
        wall.attributes["curve_length"] = curve_length

    for layer in range(num_layers):
        layer_offset = layer * (W + joint)

        for course in range(num_courses):
            # stretcher bond: odd courses offset by half a brick length
            stagger = (L + joint) / 2.0 if (course % 2 == 1) else 0.0
            z = course * (H + joint) + H / 2.0

            if curve is None:
                course_bricks = _straight_course(stagger, L, joint, num_bricks_per_course)
            else:
                course_bricks = _curve_course(curve, curve_length, stagger, L, joint)

            half_point = len(course_bricks) // 2

            for position, (point, xaxis, yaxis) in enumerate(course_bricks):
                origin = add_vectors(point, scale_vector(yaxis, layer_offset))
                origin = add_vectors(origin, [0.0, 0.0, z])
                frame = Frame(origin, xaxis, yaxis)
                half = "bad" if position < half_point else "good"

                slot = part.copy()
                slot.frame = frame
                slot.attributes.update({
                    "name": "slot_l{:02d}_c{:02d}_p{:02d}".format(layer, course, position),
                    "layer": layer,
                    "course": course,
                    "position": position,
                    "half": half,
                    "outward_axis": outward_axis,
                    "outward_side": outward_side,
                    "occupant": None,  # filled in later by allocate.py
                })
                wall.add_part(slot)

    return wall


def slots_by_half(wall):
    """Split a wall's slot parts into (bad_slots, good_slots) lists,
    each ordered by (layer, course, position).
    """
    bad, good = [], []
    for part in wall.parts():
        (bad if part.attributes["half"] == "bad" else good).append(part)
    key = lambda p: (p.attributes["layer"], p.attributes["course"], p.attributes["position"])
    bad.sort(key=key)
    good.sort(key=key)
    return bad, good


if __name__ == "__main__":
    from ..cellularized_part import CellularizedPart

    brick = CellularizedPart.from_box((0.25, 0.12, 0.065))
    wall = build_stretcher_bond_wall(brick)
    print(wall)
    bad, good = slots_by_half(wall)
    print("bad-half slots:", len(bad), "good-half slots:", len(good))
    print("example slot attrs:", bad[0].attributes)
