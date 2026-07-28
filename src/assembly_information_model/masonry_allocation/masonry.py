"""
masonry.py
==========

A simple stretcher bond wall as a container of *slots* -- positions and
orientations where a brick (a `CellularizedPart`) can be placed -- built as
an assembly_information_model `Assembly`.

This module only builds the container: an Assembly whose parts are plain
`Part` "slot" markers (frame + attributes), split into a "good" half and a
"bad" half. It does NOT decide which physical brick goes in which slot --
that's allocate.py's job, once we have a stock of graded bricks.

Stretcher bond convention used here:
- Each course (row) is offset by half a brick length from the course below.
- Bricks in a course are spaced by `joint` along X (running direction).
- Courses are stacked along Z by (height + joint).
- The wall is split into two halves along X: the first `num_bricks_per_course // 2`
  columns are the "bad" half (damaged faces oriented outward), the rest are
  the "good" half.
- "Outward" is assumed to be the -Y face of each brick (the wall's front face).
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame, Point

from ..assembly import Assembly
from ..part import Part


def build_stretcher_bond_wall(
    brick_size=(0.25, 0.12, 0.065),
    joint=0.01,
    num_courses=6,
    num_bricks_per_course=10,
    outward_axis="y",
    outward_side="-",
):
    """Build an empty stretcher bond wall as an Assembly of slot Parts.

    Parameters
    ----------
    brick_size : tuple[float, float, float]
        (length, width, height) of a single brick, in meters.
    joint : float
        Mortar/gap thickness between bricks, in meters.
    num_courses : int
        Number of rows stacked in Z.
    num_bricks_per_course : int
        Number of brick slots per row. Must be even so the wall splits
        evenly into a "good" and a "bad" half.
    outward_axis, outward_side : str
        Which local face of each brick is considered "outward" / visible
        on the wall's display side (used later by allocate.py to score
        candidate bricks). Default: the brick's local -Y face.

    Returns
    -------
    :class:`assembly_information_model.Assembly`
        The wall container. Each part has these attributes set:
        - "course": int, row index (0 = bottom)
        - "position": int, column index within the course
        - "half": "bad" | "good"
        - "outward_axis", "outward_side": str, str (copied from arguments)
    """
    if num_bricks_per_course % 2 != 0:
        raise ValueError("num_bricks_per_course must be even to split into two equal halves.")

    L, W, H = brick_size
    half_point = num_bricks_per_course // 2

    wall = Assembly(name="stretcher_bond_wall")
    wall.attributes["brick_size"] = tuple(brick_size)
    wall.attributes["joint"] = joint
    wall.attributes["num_courses"] = num_courses
    wall.attributes["num_bricks_per_course"] = num_bricks_per_course

    for course in range(num_courses):
        # stretcher bond: odd courses offset by half a brick length
        offset = (L + joint) / 2.0 if (course % 2 == 1) else 0.0
        z = course * (H + joint) + H / 2.0

        for position in range(num_bricks_per_course):
            x = offset + position * (L + joint) + L / 2.0
            y = 0.0

            frame = Frame(Point(x, y, z), [1, 0, 0], [0, 1, 0])
            half = "bad" if position < half_point else "good"

            slot = Part(
                name="slot_c{:02d}_p{:02d}".format(course, position),
                frame=frame,
                course=course,
                position=position,
                half=half,
                outward_axis=outward_axis,
                outward_side=outward_side,
                brick_size=tuple(brick_size),
                occupant=None,  # filled in later by allocate.py
            )
            wall.add_part(slot)

    return wall


def slots_by_half(wall):
    """Split a wall's slot parts into (bad_slots, good_slots) lists,
    each ordered by (course, position).
    """
    bad, good = [], []
    for part in wall.parts():
        (bad if part.attributes["half"] == "bad" else good).append(part)
    key = lambda p: (p.attributes["course"], p.attributes["position"])
    bad.sort(key=key)
    good.sort(key=key)
    return bad, good


if __name__ == "__main__":
    wall = build_stretcher_bond_wall()
    print(wall)
    bad, good = slots_by_half(wall)
    print("bad-half slots:", len(bad), "good-half slots:", len(good))
    print("example slot attrs:", bad[0].attributes)
