"""
allocate.py
===========

Baseline greedy allocator: given a wall container (masonry.py) with slots
split into "good" and "bad" halves, and a stock of graded bricks
(`CellularizedPart` instances, e.g. from a real `BrickDamageGrader` pipeline),
assigns each slot the best-fitting available brick.

Greedy strategy
----------------
1. For every stock brick, score its candidate *outward face* (the face that
   would be visible on the wall, per the slot's outward_axis/outward_side)
   -- this is `CellularizedPart.face_damage(axis, side)`.
2. Sort the stock by that outward-face damage score.
3. The "good" half gets the least-damaged bricks (lowest outward-face score),
   the "bad" half gets the most-damaged (highest outward-face score).
4. Assign in order; record the brick's key on the slot's "occupant" attribute
   and transform a copy of the brick to the slot's frame.

This is intentionally simple -- no optimization over brick-to-brick fit,
coursing tolerances, etc. It's meant to validate the wall/Part/CellNetwork
plumbing and give an immediately visible "good side vs bad side" result.
A smarter assignment (e.g. Hungarian matching under additional constraints)
can replace step 2-4 later without touching the wall or brick data structures.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Translation

from .masonry import slots_by_half


def allocate(wall, stock):
    """Assign stock bricks to wall slots in place.

    Parameters
    ----------
    wall : :class:`assembly_information_model.Assembly`
        Built by `build_stretcher_bond_wall`.
    stock : list[CellularizedPart]
        Graded brick stock, e.g. from a `BrickDamageGrader` pipeline.

    Returns
    -------
    dict
        {"placed": list[(slot_part, brick_part)], "unused": list[CellularizedPart]}
        `brick_part` in each pair is a *copy* of the stock brick, transformed
        into the slot's frame; the original stock list is left untouched.
    """
    bad_slots, good_slots = slots_by_half(wall)
    num_needed = len(bad_slots) + len(good_slots)
    if len(stock) < num_needed:
        raise ValueError(
            "Not enough stock bricks ({}) for the wall's {} slots.".format(len(stock), num_needed)
        )

    # score every stock brick by its candidate outward face
    axis = wall.attributes.get("outward_axis", "y")
    side = wall.attributes.get("outward_side", "-")
    # slots all share the same outward_axis/outward_side in this simple wall,
    # so we can read it off any slot instead of the wall attributes if needed
    if bad_slots:
        axis = bad_slots[0].attributes["outward_axis"]
        side = bad_slots[0].attributes["outward_side"]

    scored = []
    for brick in stock:
        score = brick.face_damage(axis, side)
        if score is None:
            score = brick.mean_damage() or 0.0
        scored.append((score, brick))
    scored.sort(key=lambda pair: pair[0])  # ascending: least damaged first

    least_damaged = [b for _, b in scored[:len(good_slots)]]
    most_damaged = [b for _, b in scored[-len(bad_slots):]] if bad_slots else []
    remaining = [b for _, b in scored[len(good_slots):len(scored) - len(bad_slots)]]

    placed = []

    def place(slot, brick):
        occupant = brick.copy()
        T = Translation.from_vector([
            slot.frame.point.x - occupant.frame.point.x,
            slot.frame.point.y - occupant.frame.point.y,
            slot.frame.point.z - occupant.frame.point.z,
        ])
        occupant.transform(T)
        slot.attributes["occupant"] = brick.attributes.get("stock_index", brick.attributes.get("name"))
        placed.append((slot, occupant))

    for slot, brick in zip(good_slots, least_damaged):
        place(slot, brick)
    for slot, brick in zip(bad_slots, most_damaged):
        place(slot, brick)

    used_ids = {id(b) for b in least_damaged} | {id(b) for b in most_damaged}
    unused = [b for b in stock if id(b) not in used_ids]

    return {"placed": placed, "unused": unused}
