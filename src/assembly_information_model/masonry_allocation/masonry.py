"""
masonry.py
==========

A simple brick wall as a container of *slots* -- positions and orientations
where a brick can be placed -- built as an assembly_information_model
`Assembly`.

This module only builds the container: an Assembly of slot parts (frame +
course/position/layer/front_axis/back_axis attributes), each a
positioned copy of a given template brick `Part` (typically a
`CellularizedPart`). It does NOT decide which slots are "good" vs "bad", nor
which *stock* brick ends up in which slot -- both are allocate.py's job:
`label_bad_good` assigns a "half" attribute per some pluggable criterion
(position-based, a side of the wall, an arbitrary area -- anything expressible
as a predicate), and `allocate` creates its own transformed copies of graded
stock bricks and assigns them to slots accordingly.

Each slot's `front_axis`/`back_axis` name the brick-local axis ("x" or "y")
whose face is exposed on that slot's own front/back, or `None` if that side
isn't exposed at all -- see `front_axis`/`back_axis` per bond type below.
These describe the brick's own orientation only, not whether that particular
`layer` sits at the wall's true outer boundary (relevant once `num_layers`
> 1) -- that's left for `label_bad_good`/`allocate` to decide later.

Bond conventions used here (`bond_type`):
- "stretcher" (default): brick length runs along the course, so the long
  L x H face is exposed outward, spaced/staggered by (length + joint).
  Only one face is ever exposed this way, so `front_axis` is "y" and
  `back_axis` is `None`.
- "header": brick width runs along the course instead, so the short W x H
  end face is exposed outward, spaced/staggered by (width + joint). A
  header brick ties through its own layer slot, so *both* its ends are
  exposed faces -- `front_axis` and `back_axis` are both "x".
- "flemish": alternates a header and a *stretcher pair* along the course --
  one header brick (its length filling the wall depth), then two stretcher
  bricks stacked front-to-back (each only its width deep), so the pair fills
  the same depth as the header does. Both units sit flush with the wall's
  outer face and extend back by their own depth, so headers and stretcher
  pairs line up at the front regardless of which is deeper. This means one
  Flemish course is already a full wall thickness on its own -- unlike
  stretcher/header bond, where thickness comes from `num_layers` -- so "one
  layer" of Flemish bond is those 2 stretchers + 1 header, not a single
  wythe. The header brick gets `front_axis`/`back_axis` "x"/"x"; of the
  stretcher pair, the front one gets "y"/`None` and the back one gets
  `None`/"y" (mixed within the same course).
- "english": alternates whole *courses* instead of bricks within one --
  every even course is entirely headers (:func:`_header_bond_course`), every
  odd course is entirely stretchers, doubled front-to-back
  (:func:`_stretcher_bond_course` plus :func:`_double_stretcher_course`) so
  the stretcher course's combined depth matches a header course's. So here
  too, "one layer" is really 1 header course + 2 stretcher courses' worth of
  bricks, not a single wythe -- there is no separate `_english_bond_course`;
  `build_wall` just calls the stretcher/header functions directly, course by
  course. Stretcher courses are staggered by `(W - L) / 2` so the first
  stretcher centers on the first header (that offset always works out
  non-negative -- `stagger + L/2 = W/2` -- regardless of L/W/joint); every
  other stretcher then lands on every other header, the same period-based
  relationship as Flemish bond's within-course stagger, exact for the
  standard L = 2W + joint brick-module proportion. Every header course also
  drops its own last brick (`course_bricks[:-1]`), trimming the trailing
  header that would otherwise sit past where the stretcher courses provide
  it a matching bond partner. As with Flemish's stretcher pairs, doubling
  splits `front_axis`/`back_axis` "y"/`None` for the front copy and
  `None`/"y" for the back copy.

All bond types share these rules:
- Each course (row) is offset by half a brick-along-the-course-dimension
  from the course below, so vertical joints don't line up between courses.
- Courses are stacked along world Z by (height + joint).
- Layers (wythes) are stacked along the running direction's in-plane
  perpendicular by (width + joint); each layer repeats the same course/
  position/bond pattern, just offset sideways.
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

from shapely.geometry import Polygon

from ..assembly import Assembly


def _course_anchors(course, spacing, joint, num_bricks_per_course, curve, curve_length, stagger=None):
    """(point, running_axis, depth_axis) for each anchor in one course.

    Anchors are spaced by (`spacing` + `joint`) along the course -- `spacing`
    is whichever brick dimension runs along the course (length for stretcher
    bond, width for header bond) -- and odd courses are staggered by half
    that spacing so vertical joints don't line up between courses.
    `running_axis`/`depth_axis` are unit vectors: `running_axis` along the
    course's direction of travel (world X, or the curve's local tangent),
    `depth_axis` perpendicular to it in the horizontal plane (for layer
    offsetting). Which brick axis ends up pointing where (i.e. bond type) is
    decided by the caller. Dispatches to a straight run along world X, or a
    fit along `curve`.

    stagger : float, optional
        Override the default half-`spacing` odd/even stagger with an
        explicit value. Needed when the caller's stagger requirement isn't
        "half of *this* course's own spacing" -- e.g. `bond_type="english"`,
        whose stretcher courses must align to the *header* course's rhythm,
        not their own.
    """
    anchors = []
    if stagger is None:
        stagger = (spacing + joint) / 2.0 if (course % 2 == 1) else 0.0
    if curve is None:
        for position in range(num_bricks_per_course):
            x = stagger + position * (spacing + joint) + spacing / 2.0
            anchors.append((Point(x, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]))
    else:
        position = 0
        while True:
            s = stagger + position * (spacing + joint) + spacing / 2.0
            if s + spacing / 2.0 > curve_length:
                break
            t = s / curve_length
            point = curve.point_at(t)
            running_axis = normalize_vector(curve.tangent_at(t))
            depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))
            anchors.append((point, running_axis, depth_axis))
            position += 1
    return anchors


def _stretcher_bond_course(course, L, W, joint, num_bricks_per_course, curve, curve_length, stagger=None):
    """(point, xaxis, yaxis, depth_axis, front_axis, back_axis) for each brick in one course, laid as stretchers.

    Brick length runs along the course (xaxis = running direction), width
    points into the wall depth (yaxis) -- the long L x H face ends up
    exposed on the brick's own front only (`front_axis` "y", `back_axis`
    `None`). `depth_axis` (for layer/wythe offsetting) is the same as
    `yaxis` here, but is returned separately since that stops being true for
    header bond -- see :func:`_header_bond_course`. `W` is unused (accepted
    only so all three `*_bond_course` functions share one call signature).
    `stagger` overrides the default odd/even course stagger -- see
    :func:`_course_anchors`.
    """
    anchors = _course_anchors(course, L, joint, num_bricks_per_course, curve, curve_length, stagger=stagger)
    return [(point, running, depth, depth, "y", None) for point, running, depth in anchors]


def _header_bond_course(course, L, W, joint, num_bricks_per_course, curve, curve_length, stagger=None):
    """(point, xaxis, yaxis, depth_axis, front_axis, back_axis) for each brick in one course, laid as headers.

    Brick width runs along the course, length points into the wall depth
    instead -- the short W x H end face ends up exposed at both ends of the
    brick's own layer slot (`front_axis` and `back_axis` both "x"), since a
    header ties all the way through. `xaxis` is swapped to `depth_axis` and
    `yaxis` to `-running_axis` (rather than `running_axis`/`depth_axis`
    directly, as in stretcher bond) so the frame stays right-handed with
    zaxis still pointing up. `depth_axis` is returned on its own (not just
    inferred from `yaxis`/`xaxis`) so layer offsetting always moves wythes
    apart in the true wall-depth direction, regardless of how the brick
    itself got rotated for this bond type. `L` is unused (accepted only so
    all three `*_bond_course` functions share one call signature). `stagger`
    overrides the default odd/even course stagger -- see :func:`_course_anchors`.
    """
    anchors = _course_anchors(course, W, joint, num_bricks_per_course, curve, curve_length, stagger=stagger)
    return [(point, depth, scale_vector(running, -1.0), depth, "x", "x") for point, running, depth in anchors]


def _flemish_bond_course(course, L, W, joint, num_bricks_per_course, curve, curve_length):
    """(point, xaxis, yaxis, depth_axis, front_axis, back_axis) for each PHYSICAL brick in one course, Flemish-bonded.

    Alternates two kinds of *unit* along the course:
    - a header unit: one brick, its length (L) filling the wall depth, W+joint
      of along-course footprint -- like :func:`_header_bond_course`.
    - a stretcher unit: two bricks stacked front-to-back in depth (each W
      deep), L+joint of along-course footprint -- like
      :func:`_stretcher_bond_course`, but doubled in depth so the pair fills
      the same depth as one header.
    Both kinds of unit are placed flush with the wall's outer face (`point`,
    from :func:`_course_anchors`/the curve, is treated as that face, depth 0)
    and extend backward from there by their own depth -- so headers and
    stretcher pairs align at the front even though one is deeper than the
    other. Even courses start with a stretcher pair, unstaggered; odd courses
    start with a header instead, staggered by `(L - W) / 2` (always
    non-negative since a brick's length exceeds its width). That specific
    combination -- not just an arbitrary offset -- makes every unit in an odd
    course centered exactly on the opposite-type unit in the course below/
    above it: solving "odd course's header center == even course's stretcher
    center" for the same pair index gives `(L - W) / 2` directly, and the
    same value also satisfies "odd course's stretcher center == even
    course's header center" for every later pair, for any L/W/joint (proven
    algebraically, not just for one brick's proportions).

    `num_bricks_per_course` counts *units* here, not physical bricks -- a
    stretcher unit contributes two entries to the returned list, so its
    length can exceed `num_bricks_per_course` (and needn't be even).
    """
    if course % 2 == 0:
        stagger = 0.0
        odd_first_header = False  # even courses start with a stretcher pair
    else:
        stagger = (L - W) / 2.0
        odd_first_header = True  # odd courses start with a header

    def unit_bricks(point, running_axis, depth_axis, is_header):
        if is_header:
            center = add_vectors(point, scale_vector(depth_axis, L / 2.0))
            return [(center, depth_axis, scale_vector(running_axis, -1.0), depth_axis, "x", "x")]
        front = add_vectors(point, scale_vector(depth_axis, W / 2.0))
        back = add_vectors(point, scale_vector(depth_axis, W / 2.0 + W + joint))
        return [
            (front, running_axis, depth_axis, depth_axis, "y", None),
            (back, running_axis, depth_axis, depth_axis, None, "y"),
        ]

    bricks = []
    s = stagger
    unit_index = 0
    while curve is not None or unit_index < num_bricks_per_course:
        is_header = (unit_index % 2 == 0) == odd_first_header
        footprint = W if is_header else L
        center_s = s + footprint / 2.0

        if curve is None:
            point, running_axis, depth_axis = Point(center_s, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
        else:
            if center_s + footprint / 2.0 > curve_length:
                break
            t = center_s / curve_length
            point = curve.point_at(t)
            running_axis = normalize_vector(curve.tangent_at(t))
            depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))

        bricks.extend(unit_bricks(point, running_axis, depth_axis, is_header))
        s += footprint + joint
        unit_index += 1

    return bricks


def _double_stretcher_course(course_bricks, W, joint):
    """Duplicate a stretcher course front-to-back, to fill a header's depth.

    Used for `bond_type="english"`'s stretcher courses: `course_bricks`
    (from :func:`_stretcher_bond_course`) gives each brick's own center,
    already centered on its anchor the same way :func:`_header_bond_course`
    centers a header on its own anchor (spanning `[-L/2, +L/2]`). To match
    that, the front/back pair here is placed symmetrically too -- at
    `-(W+joint)/2` and `+(W+joint)/2` from the original center, not `0` and
    `+(W+joint)` -- otherwise the pair's combined depth span ends up shifted
    backward relative to a header's, instead of centered the same way.
    `front_axis`/`back_axis` are reassigned per copy ("y"/`None` for the
    front one, `None`/"y" for the back one) rather than carried over from
    `course_bricks`, same as Flemish's stretcher pairs.
    """
    half_gap = (W + joint) / 2.0
    doubled = []
    for point, xaxis, yaxis, depth_axis, _front_axis, _back_axis in course_bricks:
        front = add_vectors(point, scale_vector(depth_axis, -half_gap))
        back = add_vectors(point, scale_vector(depth_axis, half_gap))
        doubled.append((front, xaxis, yaxis, depth_axis, "y", None))
        doubled.append((back, xaxis, yaxis, depth_axis, None, "y"))
    return doubled


def build_wall(
    part,
    joint=0.01,
    num_courses=6,
    num_bricks_per_course=10,
    num_layers=1,
    bond_type="stretcher",
    curve=None,
):
    """Build an empty brick wall as an Assembly of positioned brick copies.

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
        Number of brick slots per row. Ignored if `curve` is given -- see the
        module docstring.
    num_layers : int, optional
        Number of parallel wythes, stacked along the in-plane perpendicular
        to the running direction by (width + joint). Default 1 (single layer).
    bond_type : {"stretcher", "header", "flemish", "english"}, optional
        Which of :func:`_stretcher_bond_course` / :func:`_header_bond_course`
        / :func:`_flemish_bond_course` every layer/course uses, or (for
        "english") alternates course by course -- see the module docstring.
    curve : :class:`compas.geometry.Curve` | :class:`compas.geometry.Polyline`, optional
        If given, courses follow this curve (from its start point) instead of
        a straight line along world X -- see the module docstring for the
        fitting/orientation rules.

    Returns
    -------
    :class:`assembly_information_model.Assembly`
        The wall container. Each part is a copy of `part`, repositioned to
        its slot, with these attributes set:
        - "layer": int, wythe index (0 = first)
        - "course": int, row index (0 = bottom)
        - "position": int, column index within the course
        - "front_axis", "back_axis": str or None, str or None -- the
          brick-local axis ("x"/"y") exposed on that slot's own front/back,
          per the bond type -- see the module docstring. `None` means that
          side isn't exposed at all.
        - "occupant": None, filled in later by allocate.py

        No "half" attribute yet -- see `allocate.label_bad_good` to assign
        one before calling `allocate.allocate`.
    """
    course_bond_funcs = {
        "stretcher": _stretcher_bond_course,
        "header": _header_bond_course,
        "flemish": _flemish_bond_course,
        "english": None,  # no single course function -- see the per-course dispatch below
    }
    if bond_type not in course_bond_funcs:
        raise ValueError("bond_type must be one of {}, got {!r}.".format(list(course_bond_funcs), bond_type))

    box = part.mesh.aabb()
    L, W, H = box.xsize, box.ysize, box.zsize
    course_bond = course_bond_funcs[bond_type]
    # depth: whichever dimension runs *into* the wall -- what extra
    # `num_layers` stack by. Header bond rotates the brick 90 deg so its
    # length becomes the depth; Flemish's/English's header-plus-stretcher-
    # pair unit/course is already a full L deep on its own (the point of
    # both bonds), same depth footprint as header bond.
    depth = {"stretcher": W, "header": L, "flemish": L, "english": L}[bond_type]

    curve_length = None
    if curve is not None:
        if not isinstance(curve, Polyline):
            curve = curve.to_polyline()
        curve_length = curve.length

    wall = Assembly(name="wall")
    wall.attributes["brick_size"] = (L, W, H)
    wall.attributes["joint"] = joint
    wall.attributes["num_courses"] = num_courses
    wall.attributes["num_layers"] = num_layers
    wall.attributes["bond_type"] = bond_type
    if curve is None:
        wall.attributes["num_bricks_per_course"] = num_bricks_per_course
    else:
        wall.attributes["curve_length"] = curve_length

    for layer in range(num_layers):
        layer_offset = layer * (depth + joint)

        for course in range(num_courses):
            z = course * (H + joint) + H / 2.0

            if bond_type == "english":
                if course % 2 == 0:
                    # every header course repeats identically -- no stagger
                    # between them (they're 2 rows apart, a stretcher course
                    # sits between); `_header_bond_course`'s own odd/even
                    # stagger would only matter between ADJACENT header
                    # courses, which never happens here.
                    course_bricks = _header_bond_course(
                        course, L, W, joint, num_bricks_per_course, curve, curve_length, stagger=0.0
                    )
                    if course_bricks:
                        course_bricks = course_bricks[:-1]
                else:
                    # centers the FIRST stretcher on the FIRST header:
                    # stagger + L/2 = (W-L)/2 + L/2 = W/2, exactly the first
                    # header's own center -- always positive regardless of
                    # L/W/joint, so this never pushes a brick before the
                    # course start. (Every other stretcher then lands on
                    # every other header, same period-based relationship as
                    # Flemish bond's cross-brick stagger, just phased to
                    # start matching from the first header instead of a
                    # later one.)
                    front = _stretcher_bond_course(
                        course, L, W, joint, num_bricks_per_course, curve, curve_length, stagger=(W - L) / 2.0
                    )
                    course_bricks = _double_stretcher_course(front, W, joint)
            else:
                course_bricks = course_bond(course, L, W, joint, num_bricks_per_course, curve, curve_length)

            for position, (point, xaxis, yaxis, depth_axis, front_axis, back_axis) in enumerate(course_bricks):
                origin = add_vectors(point, scale_vector(depth_axis, layer_offset))
                origin = add_vectors(origin, [0.0, 0.0, z])
                # cross(xaxis, yaxis) points up by construction everywhere above;
                # flip it to point down by negating whichever of the two axes
                # ISN'T the brick's own front-facing one, so front_axis/
                # back_axis keep referring to the exact same physical face.
                if front_axis == "x":
                    frame = Frame(origin, xaxis, scale_vector(yaxis, -1.0))
                else:
                    frame = Frame(origin, scale_vector(xaxis, -1.0), yaxis)

                slot = part.copy()
                slot.frame = frame
                slot.attributes.update({
                    "name": "slot_l{:02d}_c{:02d}_p{:02d}".format(layer, course, position),
                    "layer": layer,
                    "course": course,
                    "position": position,
                    "front_axis": front_axis,
                    "back_axis": back_axis,
                    "front_label": None, # filled in later by label.py
                    "back_label": None,  # filled in later by label.py
                    "occupant": None,  # filled in later by allocate.py
                })
                wall.add_part(slot)

    return wall


def _footprint(part, L, W):
    """The world-space horizontal (X-Y) footprint of a wall part, as a shapely Polygon.

    Since a slot's `mesh`/`shape` is deliberately left untransformed (only
    `frame` marks its actual position -- see the module docstring),
    reconstructs the footprint analytically instead: a rectangle `L` along
    `frame.xaxis` by `W` along `frame.yaxis`, centered on `frame.point`. This
    holds regardless of `bond_type`, since every bond just points the
    template's own fixed local L/W axes in different world directions --
    it never changes which local axis is L and which is W.
    """
    dx = scale_vector(part.frame.xaxis, L / 2.0)
    dy = scale_vector(part.frame.yaxis, W / 2.0)
    corners = [
        add_vectors(add_vectors(part.frame.point, scale_vector(dx, sx)), scale_vector(dy, sy))
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))
    ]
    return Polygon([(c[0], c[1]) for c in corners])


def connect_wall(wall, min_overlap_area=1e-6):
    """Connect each brick to whichever brick(s) sit directly below it.

    `interfaces.assembly_interfaces_numpy` finds connections by testing for
    actual mesh-face contact, which never registers here: the mortar `joint`
    deliberately keeps bricks apart, and it also relies on an older
    `assembly.network` attribute this `Assembly` no longer has. Since the
    wall's exact layout is already known, this instead computes each brick's
    exact world-space horizontal footprint directly from its `frame` (see
    :func:`_footprint`) and connects any pair, in the same `layer` and
    adjacent `course`s, whose footprints overlap by at least
    `min_overlap_area` -- a brick can end up connected to one or two bricks
    below it (e.g. whenever a course is staggered relative to the one below).

    Parameters
    ----------
    wall : :class:`assembly_information_model.Assembly`
        Built by :func:`build_wall`.
    min_overlap_area : float, optional
        Minimum horizontal overlap area (wall units squared) to count as
        "resting on".

    Returns
    -------
    int
        Number of connections added.
    """
    L, W, _H = wall.attributes["brick_size"]

    by_layer_course = {}
    for part in wall.parts():
        key = (part.attributes["layer"], part.attributes["course"])
        by_layer_course.setdefault(key, []).append(part)

    count = 0
    for (layer, course), parts_above in by_layer_course.items():
        parts_below = by_layer_course.get((layer, course - 1))
        if not parts_below:
            continue

        below_footprints = [(below, _footprint(below, L, W)) for below in parts_below]
        for above in parts_above:
            fp_above = _footprint(above, L, W)
            for below, fp_below in below_footprints:
                if fp_above.intersects(fp_below) and fp_above.intersection(fp_below).area >= min_overlap_area:
                    wall.add_connection(below, above, interface_type="course_support")
                    count += 1

    return count


if __name__ == "__main__":
    from ..cellularized_part import CellularizedPart

    brick = CellularizedPart.from_box((0.25, 0.12, 0.065))
    wall = build_wall(brick)
    print(wall)
    n = connect_wall(wall)
    print("connections added:", n)
    print("example slot attrs:", next(wall.parts()).attributes)
