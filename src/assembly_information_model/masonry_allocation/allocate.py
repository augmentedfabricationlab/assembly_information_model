"""
allocate.py
===========

Given a `container` assembly (masonry.py) with some slots labeled by
`label.label_facade` (a "front_label"/"back_label" -- "bad"/"good"/`None` --
on every slot), and a `stock` assembly of graded bricks (`CellularizedPart`
instances, e.g. from a real `BrickDamageGrader` pipeline / "Make Material
Stock"), `allocate(container, stock)` assigns stock bricks to slots and
returns a new assembly.

Only labeled, exposed sides constrain matching -- a slot with `front_axis`
set but no `front_label` (not labeled), or `front_axis=None` (not exposed
there at all), imposes no requirement on that side. A slot can end up with
zero requirements (any leftover brick will do), one (e.g. a stretcher, whose
`back_axis` is always `None`), or two (e.g. a header, exposed -- and
possibly labeled differently -- on both faces).

Bricks are free to flip when placed
--------------------------------------
`masonry.py` no longer tracks which literal end (`+`/`-`) of a slot's
exposed axis is "outward" -- only which axis. That's deliberate: a stock
brick isn't stuck presenting one fixed end either. For any axis, a brick has
two end-damage scores (`face_damage(axis, "-")` and `face_damage(axis,
"+")`), and allocation is free to choose which one faces which way when it
places that brick -- i.e. to flip it 180 degrees around its own vertical
axis. So a single-sided (stretcher) slot just needs *one* end to fit; a
dual-sided (header) slot needs *both* ends to fit *simultaneously*, in
whichever of the two orientations fits best -- these aren't independent
choices, since they're the two ends of the one axis.

Greedy strategy
---------------
1. For every constrained slot, and every remaining stock brick, score both
   possible orientations by summing, over the slot's requirement(s), how far
   that orientation's side-score is from the label's target (`bad` -> 1.0,
   `good` -> 0.0; damage scores are in [0, 1] -- see `cellularized_part.py`).
   The better orientation's cost is that brick's fit for that slot.
2. Slots are settled most-constrained-first (two requirements before one),
   each taking whichever remaining brick fits it best.
3. Leftover slots (no requirement at all) and leftover stock are paired off
   in whatever order remains -- nothing to optimize there.

This is intentionally simple -- greedy, one slot at a time, no lookahead
across slots. It's meant to validate the container/Part/CellNetwork plumbing
end to end, not to be optimal. A real assignment solver (e.g. Hungarian
matching over the full slot x brick x orientation cost table) can replace
this later without touching the container or brick data structures.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.geometry import scale_vector

from ..assembly import Assembly

_TARGET = {"bad": 1.0, "good": 0.0}


def _slot_requirements(slot):
    """[(side, axis, label), ...] for `slot`'s exposed *and* labeled sides only."""
    requirements = []
    for side in ("front", "back"):
        axis = slot.attributes.get("{}_axis".format(side))
        label = slot.attributes.get("{}_label".format(side))
        if axis is not None and label is not None:
            requirements.append((side, axis, label))
    return requirements


def _brick_end_scores(brick, axis):
    """(minus, plus) damage scores of `brick`'s two `axis` ends.

    Falls back to the brick's overall `mean_damage()` (or 0.0) for whichever
    end has no graded cells of its own.
    """
    fallback = brick.mean_damage()
    if fallback is None:
        fallback = 0.0
    minus = brick.face_damage(axis, "-")
    plus = brick.face_damage(axis, "+")
    return (minus if minus is not None else fallback, plus if plus is not None else fallback)


def _fit(requirements, minus, plus):
    """Best of the two ways a brick scored (`minus`, `plus`) on `requirements`'s
    axis can be oriented into that slot.

    Returns
    -------
    (float, bool)
        (cost, flipped) -- lower cost is a better fit (0 = every requirement
        met exactly); `flipped` says whether that orientation swaps which
        end faces front vs back from `masonry.py`'s unflipped convention
        (front = local minus) -- see the module docstring.
    """
    unflipped = {"front": minus, "back": plus}
    flipped = {"front": plus, "back": minus}

    def cost(sides):
        return sum(abs(sides[side] - _TARGET[label]) for side, _axis, label in requirements)

    cost_unflipped, cost_flipped = cost(unflipped), cost(flipped)
    if cost_unflipped <= cost_flipped:
        return cost_unflipped, False
    return cost_flipped, True


def _flip_frame(frame):
    """A copy of `frame` turned 180 degrees around its own Z axis.

    Swaps which end of the brick's exposed axis (or axes) faces front vs
    back while leaving "up" unchanged -- see the module docstring on why
    bricks are free to flip like this between stock and container.
    """
    return Frame(frame.point, scale_vector(frame.xaxis, -1.0), scale_vector(frame.yaxis, -1.0))


def allocate(container, stock):
    """Assign stock bricks to container slots, in a new assembly.

    Parameters
    ----------
    container : :class:`assembly_information_model.Assembly`
        Built by `build_wall`, optionally labeled by `label.label_facade`
        (front and/or back, independently -- see the module docstring). Its
        parts are plain `Part` slots (frame plus course/position/layer/
        front_axis/back_axis/front_label/back_label attributes); left
        untouched -- only their frames/attributes are read.
    stock : :class:`assembly_information_model.Assembly`
        An assembly of graded `CellularizedPart` bricks (e.g. from a
        `BrickDamageGrader` pipeline); left untouched.

    Returns
    -------
    :class:`assembly_information_model.Assembly`
        A new assembly with one part per container slot: a copy of the
        assigned stock brick, with its `frame` set to that slot's frame (or
        a 180-degree-flipped copy of it -- see `_flip_frame` -- if that
        orientation fit better), geometry left in the brick's own local
        coordinates, same convention as `build_wall`. Each part's key equals
        the SLOT's key in `container` (not the stock brick's own key -- see
        `source_key` below) -- so `container`'s connections (e.g. from
        `connect_wall`) carry straight over onto the result unchanged,
        rather than needing `connect_wall` run again. Its `layer`/`course`/
        `position`/`front_axis`/`back_axis`/`front_label`/`back_label`
        attributes are copied over from the slot it was placed in, plus:
        - `front_damage_score`/`back_damage_score`: the achieved score on
          each required side (`None` for a side that wasn't a requirement).
        - `source_key`: the stock brick's ORIGINAL key in `stock`, for
          tracing which physical brick ended up where now that the part's
          own key tracks the slot instead.

        so :func:`summarize_allocation` (or your own queries) can group/
        inspect results by slot without having to re-match parts by frame
        position.

    Raises
    ------
    ValueError
        If `stock` doesn't have enough parts for the container's slots.
    """
    slots = sorted(
        container.parts(), key=lambda p: (p.attributes["layer"], p.attributes["course"], p.attributes["position"])
    )
    stock_parts = list(stock.parts())
    if len(stock_parts) < len(slots):
        raise ValueError(
            "Not enough stock bricks ({}) for the container's {} slots.".format(len(stock_parts), len(slots))
        )

    constrained, unconstrained = [], []
    for slot in slots:
        requirements = _slot_requirements(slot)
        (constrained if requirements else unconstrained).append((slot, requirements))

    # most-constrained (both faces labeled) first, so a brick that could
    # satisfy either a dual- or a single-sided slot goes to the harder one
    constrained.sort(key=lambda pair: -len(pair[1]))

    remaining = list(stock_parts)
    result = Assembly(name="allocated_" + (container.name or "assembly"))
    # carry over build_wall's own wall-level metadata (brick_size, joint,
    # num_courses, ...) so connect_wall (or anything else keyed off it)
    # still works on the allocated assembly, not just the container
    result.attributes.update({key: value for key, value in container.attributes.items() if key != "name"})

    def place(slot, brick, flipped, front_score, back_score):
        occupant = brick.copy()
        occupant.frame = _flip_frame(slot.frame) if flipped else slot.frame.copy()
        occupant.attributes.update(
            {
                "layer": slot.attributes["layer"],
                "course": slot.attributes["course"],
                "position": slot.attributes["position"],
                "front_axis": slot.attributes.get("front_axis"),
                "back_axis": slot.attributes.get("back_axis"),
                "front_label": slot.attributes.get("front_label"),
                "back_label": slot.attributes.get("back_label"),
                "front_damage_score": front_score,
                "back_damage_score": back_score,
                "source_key": brick.key,
            }
        )
        # same key as the slot it fills (not the stock brick's own key --
        # that's `source_key` now) so `container`'s connections carry over
        # onto `result` unchanged, below
        result.add_part(occupant, key=slot.key)

    for slot, requirements in constrained:
        axis = requirements[0][1]  # every requirement on one slot shares an axis in practice (see module docstring)
        sides = set(side for side, _axis, _label in requirements)

        best_index = best_cost = best_flipped = best_minus = best_plus = None
        for index, brick in enumerate(remaining):
            minus, plus = _brick_end_scores(brick, axis)
            cost, flipped = _fit(requirements, minus, plus)
            if best_cost is None or cost < best_cost:
                best_index, best_cost, best_flipped = index, cost, flipped
                best_minus, best_plus = minus, plus

        brick = remaining.pop(best_index)
        front_score, back_score = (best_plus, best_minus) if best_flipped else (best_minus, best_plus)
        place(
            slot,
            brick,
            best_flipped,
            front_score if "front" in sides else None,
            back_score if "back" in sides else None,
        )

    unconstrained.sort(
        key=lambda pair: (pair[0].attributes["layer"], pair[0].attributes["course"], pair[0].attributes["position"])
    )
    for (slot, _requirements), brick in zip(unconstrained, remaining):
        place(slot, brick, False, None, None)

    # container's own connections (if any -- e.g. from connect_wall) are
    # still valid as-is on result, since occupants share their slot's key
    for (u, v), attrs in container.connections(data=True):
        result.graph.add_edge(u, v, **attrs)

    return result


def summarize_allocation(result):
    """Print a per-(side, label) damage-score report for an assembly from `allocate`.

    A text alternative to eyeballing a colour-coded render: reports how many
    parts landed in each (front/back, bad/good) bucket and the min/mean/max
    achieved damage score within it. Parts with no requirement on a given
    side (unlabeled or unexposed there) don't contribute to that side's
    buckets.

    Parameters
    ----------
    result : :class:`assembly_information_model.Assembly`
        Returned by :func:`allocate`.
    """
    buckets = {}
    for part in result.parts():
        for side in ("front", "back"):
            label = part.attributes.get("{}_label".format(side))
            score = part.attributes.get("{}_damage_score".format(side))
            if label is None or score is None:
                continue
            buckets.setdefault((side, label), []).append(score)

    for (side, label), scores in sorted(buckets.items()):
        print(
            "{:>5} {:<4}: {:3d} parts | damage min={:.3f} mean={:.3f} max={:.3f}".format(
                side, label, len(scores), min(scores), sum(scores) / len(scores), max(scores)
            )
        )
