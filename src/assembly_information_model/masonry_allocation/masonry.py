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

from compas.geometry import Frame, Point, Polyline, Box
from compas.geometry import add_vectors
from compas.geometry import scale_vector
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import Scale

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


def _stretcher_closure_brick(s, curve, curve_length):
    """(point, xaxis, yaxis, depth_axis, front_axis, back_axis) for a half brick, centered at arc-length `s` along a stretcher course.

    `s` is measured the same way `_course_anchors` measures it -- distance
    along world X (no `curve`), or along `curve` from its start point --
    so a caller can place this closure anywhere along the course by giving
    the arc-length position of its own center, the same way a regular
    brick's center is computed. See `build_wall`'s stretcher-bond dispatch
    for where `s` comes from for the leading/trailing closures specifically.
    """
    if curve is None:
        return Point(s, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], "y", None
    t = s / curve_length
    point = curve.point_at(t)
    running_axis = normalize_vector(curve.tangent_at(t))
    depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))
    return point, running_axis, depth_axis, depth_axis, "y", None


def _shrunk_copy(part, axis, fraction):
    """A copy of `part` whose mesh (and shape, if it's a `Box`) is shrunk to `fraction` of itself along its own local `axis`.

    Used for a closure brick -- its mesh/shape are otherwise identical
    copies of the template `part`'s (see `Part.copy`), which is the wrong
    *geometry* for something that's supposed to be a half brick, even once
    it's correctly positioned via `frame`. Mesh vertices (and a `Box`
    shape's own dimensions) are in the part's own local coordinates,
    centered on its local origin (see the module docstring on `frame`) --
    so scaling by `fraction` along one local axis, about that same origin,
    keeps the shrunk brick centered exactly where a regular brick would be,
    just smaller. `axis`: "x" shrinks length (L), "y" shrinks width (W) --
    whichever runs along the course for this brick's orientation, same rule
    as `_footprint`'s.
    """
    slot = part.copy()
    factors = [fraction if axis == "x" else 1.0, fraction if axis == "y" else 1.0, 1.0]
    scale = Scale.from_factors(factors)
    if slot.mesh is not None:
        slot.mesh.transform(scale)
    if isinstance(slot.shape, Box):
        box = slot.shape
        slot.shape = Box(box.xsize * factors[0], box.ysize * factors[1], box.zsize, frame=box.frame.copy())
    return slot


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


def _header_closure_brick(s, curve, curve_length):
    """(point, xaxis, yaxis, depth_axis, front_axis, back_axis) for a longitudinal half brick, centered at arc-length `s` along a header course.

    A header brick's length (L) runs into the wall depth, tying all the way
    through -- shortening *that* dimension for a closure would leave a gap
    in the bond through the wall, so a header closer is instead cut the
    other way: lengthwise, parallel to its own length, giving a brick with
    half the width (W, the dimension that runs along the course here) but
    the full length -- a "long_half" (queen closer). Otherwise identical to
    :func:`_stretcher_closure_brick`: `s` is arc-length along the course the
    same way :func:`_course_anchors` measures it, and the point/axes use the
    same xaxis/yaxis swap as :func:`_header_bond_course` so this closure's
    frame convention matches a regular header brick's.

    Unlike a regular brick's anchor, `s` here can fall outside `[0,
    curve_length]` (English bond's header-course closures extend past the
    reference courses' own ends -- see `build_wall`'s "english" dispatch).
    `curve.point_at`/`tangent_at` aren't defined out there, so this falls
    back to a straight extrapolation from whichever end `s` is past, along
    that end's own tangent -- fine since `s` is only ever a brick-scale
    distance beyond the curve, never far enough for the curve's actual
    curvature over that little distance to matter.
    """
    if curve is None:
        point, running_axis, depth_axis = Point(s, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    else:
        t = s / curve_length
        t_clamped = min(max(t, 0.0), 1.0)
        anchor = curve.point_at(t_clamped)
        running_axis = normalize_vector(curve.tangent_at(t_clamped))
        overshoot = s - t_clamped * curve_length
        point = anchor if overshoot == 0.0 else add_vectors(anchor, scale_vector(running_axis, overshoot))
        depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))
    return point, depth_axis, scale_vector(running_axis, -1.0), depth_axis, "x", "x"


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
    stagger, odd_first_header = _flemish_stagger(course, L, W)
    layout = _flemish_unit_layout(stagger, odd_first_header, num_bricks_per_course, L, W, joint, curve_length)

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
    for center_s, _footprint, is_header in layout:
        if curve is None:
            point, running_axis, depth_axis = Point(center_s, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
        else:
            t = center_s / curve_length
            point = curve.point_at(t)
            running_axis = normalize_vector(curve.tangent_at(t))
            depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))
        bricks.extend(unit_bricks(point, running_axis, depth_axis, is_header))

    return bricks


def _flemish_stagger(course, L, W):
    """(stagger, odd_first_header) for one Flemish course -- see `_flemish_bond_course`."""
    if course % 2 == 0:
        return 0.0, False  # even courses start with a stretcher pair, unstaggered
    return (L - W) / 2.0, True  # odd courses start with a header, staggered


def _flemish_unit_layout(stagger, odd_first_header, num_bricks_per_course, L, W, joint, curve_length):
    """[(center_s, footprint, is_header), ...] scalar (1D) layout for one Flemish course's units.

    Pure arc-length bookkeeping, factored out of :func:`_flemish_bond_course`
    so `build_wall` can also use it to find exactly where a course's last
    unit ends (to place a closure right after it) without duplicating this
    loop or generating any 3D geometry. Unbounded by `num_bricks_per_course`
    when `curve_length` is given -- stops when the next unit would run past
    it instead.
    """
    units = []
    s = stagger
    unit_index = 0
    while curve_length is not None or unit_index < num_bricks_per_course:
        is_header = (unit_index % 2 == 0) == odd_first_header
        footprint = W if is_header else L
        center_s = s + footprint / 2.0
        if curve_length is not None and center_s + footprint / 2.0 > curve_length:
            break
        units.append((center_s, footprint, is_header))
        s += footprint + joint
        unit_index += 1
    return units


def _flemish_pair_closure_bricks(s, curve, curve_length, W, joint):
    """[(point, xaxis, yaxis, depth_axis, front_axis, back_axis), (...)] for a stretcher-style closure pair, centered at arc-length `s` along a Flemish course.

    Mirrors the stretcher-pair branch of `_flemish_bond_course`'s own
    `unit_bricks` (front/back copies stacked `W` deep, flush with the
    wall's outer face at depth 0) as a standalone pair, sized externally
    (via `build_wall`'s `_shrunk_copy`/`course_length`) rather than a fixed
    `L`, so it works for whatever along-course fraction this particular
    closure pair needs -- e.g. the quarter-length pair `build_wall` prepends
    to an odd Flemish course's own start.
    """
    if curve is None:
        point, running_axis, depth_axis = Point(s, 0.0, 0.0), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    else:
        t = s / curve_length
        point = curve.point_at(t)
        running_axis = normalize_vector(curve.tangent_at(t))
        depth_axis = normalize_vector(cross_vectors([0.0, 0.0, 1.0], running_axis))
    front = add_vectors(point, scale_vector(depth_axis, W / 2.0))
    back = add_vectors(point, scale_vector(depth_axis, W / 2.0 + W + joint))
    return [
        (front, running_axis, depth_axis, depth_axis, "y", None),
        (back, running_axis, depth_axis, depth_axis, None, "y"),
    ]


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
        - "is_closure": bool -- True for a closure brick inserted at the
          start and/or end of a staggered stretcher/header course, of every
          header course (`bond_type="english"`), at the start of an odd
          Flemish course, or at the end of an even one (see `build_wall`'s
          "flemish" dispatch for why start and end land on opposite
          parities there), False for a regular full-size brick. A stretcher
          (or Flemish stretcher-pair) closure is cut across -- a fraction of
          L, full W; a header closure is cut lengthwise instead -- full L, a
          fraction of W -- since shortening a header's L would break the
          bond through the wall depth -- see `_header_closure_brick`.
        - "brick_fraction": float -- 1.0 for a regular brick, else the
          closure's fraction of one (0.75/0.5/0.25 for a three-quarter,
          half, or quarter).
        - "course_length": float -- this slot's actual along-course
          footprint (world units); `brick_fraction` times the regular size
          for a closure. Used by `_footprint`/`connect_wall` so a closure's
          smaller footprint isn't mistaken for a full brick's.
        - "occupant": None, filled in later by allocate.py

        A closure slot's own "name" carries a "_half_length"/"_quarter_length"/
        "_three_quarter_length" (cut across, fraction of L) or "_half_width"
        (cut lengthwise, fraction of W) suffix, and its `mesh`/`shape` are an
        actual `brick_fraction`-size copy of `part`'s -- see `_shrunk_copy`
        -- not just a smaller `frame`. No "front_label"/"back_label" yet --
        see `label.label_facade` to assign those before calling
        `allocate.allocate` (closures are skipped there regardless of style).
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
    # (closure brick constructor, along-course spacing) for bond types that
    # get start/end closures on their staggered (odd) courses -- see the
    # per-course dispatch below
    closure_bond = {"stretcher": (_stretcher_closure_brick, L), "header": (_header_closure_brick, W)}

    curve_length = None
    if curve is not None:
        if not isinstance(curve, Polyline):
            curve = curve.to_polyline()
        curve_length = curve.length

    # Flemish: which course parity's own natural unit sequence ends up
    # SHORTER gets the three-quarter-length end closure, to catch up to the
    # other parity's end -- see the per-course dispatch below. This isn't
    # fixed to odd or even: without a curve, it flips on whether
    # `num_bricks_per_course` is even or odd (an even count has equal
    # numbers of header/stretcher-pair units regardless of which starts, so
    # the two parities' totals match and only the odd course's own stagger
    # makes it the longer one -- an odd count instead gives whichever parity
    # starts with the stretcher-pair one extra L-sized unit over a
    # W-sized one, which can outweigh the stagger); with a curve, it's
    # however the alternating W/L footprints happen to pack against that
    # specific `curve_length`. So it's resolved by actually generating both
    # parities' own layouts and comparing where they end, not assumed.
    flemish_end_parity = None
    if bond_type == "flemish":
        even_layout = _flemish_unit_layout(0.0, False, num_bricks_per_course, L, W, joint, curve_length)
        odd_layout = _flemish_unit_layout((L - W) / 2.0, True, num_bricks_per_course, L, W, joint, curve_length)
        even_edge = (even_layout[-1][0] + even_layout[-1][1] / 2.0) if even_layout else 0.0
        odd_edge = (odd_layout[-1][0] + odd_layout[-1][1] / 2.0) if odd_layout else 0.0
        if even_edge < odd_edge:
            flemish_end_parity = 0
        elif odd_edge < even_edge:
            flemish_end_parity = 1

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

            # a staggered (odd) stretcher/header course is offset by half a
            # brick from the even courses -- see `_course_anchors` -- leaving
            # a gap of that same size at its own start, and (since every
            # brick after it is shifted along by that same offset) an
            # overhang of that same size past where the even courses end.
            # Close both with a half brick -- see `_stretcher_closure_brick`/
            # `_header_closure_brick`. `closure_fractions[i]` is `None` for a
            # regular brick, or `course_bricks[i]`'s fraction of a full one.
            closure_fractions = [None] * len(course_bricks)
            if bond_type in closure_bond and course % 2 == 1 and course_bricks:
                closure_brick, spacing = closure_bond[bond_type]
                start_closure = closure_brick(spacing / 4.0, curve, curve_length)

                stagger = (spacing + joint) / 2.0
                n = len(course_bricks)
                last_edge = stagger + (n - 1) * (spacing + joint) + spacing  # right edge of the last full brick
                end_s = last_edge + joint + spacing / 4.0
                add_end_closure = curve is None or end_s + spacing / 4.0 <= curve_length

                course_bricks = [start_closure] + course_bricks
                closure_fractions = [0.5] + closure_fractions
                if add_end_closure:
                    course_bricks = course_bricks + [closure_brick(end_s, curve, curve_length)]
                    closure_fractions = closure_fractions + [0.5]
            elif bond_type == "english" and course % 2 == 0 and course_bricks:
                # English's header courses aren't staggered (stagger=0.0
                # above) -- they're already the reference courses everything
                # else lines up flush with -- but still get a half-width
                # closure at each end, same cut as a plain header bond
                # closure (see `_header_closure_brick`), extending the
                # course itself rather than filling a gap. With a `curve`,
                # the start closure falls before the curve's own start
                # (arc-length s < 0) -- `_header_closure_brick` extrapolates
                # for that.
                n = len(course_bricks)
                edge_last = (n - 1) * (W + joint) + W  # right edge of the last header, stagger 0
                start_s = -(joint + W / 4.0)
                end_s = edge_last + joint + W / 4.0
                add_end_closure = curve is None or end_s + W / 4.0 <= curve_length

                course_bricks = [_header_closure_brick(start_s, curve, curve_length)] + course_bricks
                closure_fractions = [0.5] + closure_fractions
                if add_end_closure:
                    course_bricks = course_bricks + [_header_closure_brick(end_s, curve, curve_length)]
                    closure_fractions = closure_fractions + [0.5]
            elif bond_type == "flemish" and course_bricks:
                # odd Flemish courses start with a header, staggered by
                # (L - W) / 2 (see `_flemish_bond_course`), leaving a gap at
                # the very start -- close it with a quarter-length stretcher
                # pair flush against the course's own start, mirroring a
                # regular stretcher unit but at 1/4 of L instead of a full
                # one. This half is always on the odd course.
                if course % 2 == 1:
                    start_pair = _flemish_pair_closure_bricks(L / 8.0, curve, curve_length, W, joint)
                    course_bricks = start_pair + course_bricks
                    closure_fractions = [0.25, 0.25] + closure_fractions

                # At the end, it's whichever course parity's own natural
                # sequence falls SHORT of the other's (`flemish_end_parity`,
                # computed once above by actually comparing both) that gets
                # a three-quarter-length stretcher pair, right after its own
                # last unit (found via `_flemish_unit_layout`, the same
                # scalar layout `_flemish_bond_course` itself used to build
                # `course_bricks`), to roughly catch up -- same "add to
                # whichever side falls short" logic as the start closure,
                # just at the opposite end, and NOT necessarily the same
                # parity as the start closure -- see `flemish_end_parity`.
                if course % 2 == flemish_end_parity:
                    stagger, odd_first_header = _flemish_stagger(course, L, W)
                    layout = _flemish_unit_layout(
                        stagger, odd_first_header, num_bricks_per_course, L, W, joint, curve_length
                    )
                    if layout:
                        last_s, last_footprint, _last_is_header = layout[-1]
                        edge_last = last_s + last_footprint / 2.0
                        end_s = edge_last + joint + 3.0 * L / 8.0
                        add_end_closure = curve is None or end_s + 3.0 * L / 8.0 <= curve_length
                        if add_end_closure:
                            end_pair = _flemish_pair_closure_bricks(end_s, curve, curve_length, W, joint)
                            course_bricks = course_bricks + end_pair
                            closure_fractions = closure_fractions + [0.75, 0.75]

            for position, ((point, xaxis, yaxis, depth_axis, front_axis, back_axis), fraction) in enumerate(
                zip(course_bricks, closure_fractions)
            ):
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

                is_closure = fraction is not None
                if is_closure:
                    # whichever of L/W runs along the course for THIS brick's
                    # own orientation (W for a header-type face, L
                    # otherwise) -- see `_footprint`'s identical rule
                    shrink_axis = "y" if front_axis == "x" else "x"
                    slot = _shrunk_copy(part, shrink_axis, fraction)
                else:
                    slot = part.copy()
                slot.frame = frame

                full_spacing = W if front_axis == "x" else L
                course_length = (full_spacing * fraction) if is_closure else full_spacing
                name = "slot_l{:02d}_c{:02d}_p{:02d}".format(layer, course, position)
                if is_closure:
                    # a header-oriented closure is cut lengthwise (full L,
                    # a fraction of W) -- since shortening L would break the
                    # bond through the wall depth -- named "..._width";
                    # any other closure is cut across (a fraction of L,
                    # full W) instead, named "..._length"
                    frac_name = {0.75: "three_quarter", 0.5: "half", 0.25: "quarter"}.get(
                        fraction, "{:g}x".format(fraction)
                    )
                    dim_name = "width" if front_axis == "x" else "length"
                    name += "_{}_{}".format(frac_name, dim_name)

                slot.attributes.update({
                    "name": name,
                    "layer": layer,
                    "course": course,
                    "position": position,
                    "front_axis": front_axis,
                    "back_axis": back_axis,
                    "front_label": None, # filled in later by label.py
                    "back_label": None,  # filled in later by label.py
                    "occupant": None,  # filled in later by allocate.py
                    "is_closure": is_closure,
                    "brick_fraction": fraction if is_closure else 1.0,
                    "course_length": course_length,
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
    it never changes which local axis is L and which is W -- EXCEPT for a
    closure brick (`part.attributes["is_closure"]`), whose along-course
    dimension is shrunk to its own "course_length" (see `build_wall`)
    instead of the template's full L or W: along `frame.yaxis` (W) for a
    header-oriented closure (`front_axis` "x"), along `frame.xaxis` (L)
    otherwise -- matching how `build_wall` placed it.
    """
    if part.attributes.get("is_closure"):
        course_length = part.attributes["course_length"]
        if part.attributes.get("front_axis") == "x":
            W = course_length
        else:
            L = course_length
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
