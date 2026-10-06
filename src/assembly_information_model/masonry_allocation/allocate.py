"""
allocate.py
===========

Given a `container` assembly (masonry.py) with some slots labeled by
`label.label_facade` (a "front_label"/"back_label" -- a target in `[0, 1]`,
or `None` -- on every slot), and a `stock` assembly of graded bricks
(`CellularizedPart` instances, e.g. from a real `BrickDamageGrader` pipeline
/ "Make Material Stock"), `allocate(container, stock)` assigns stock bricks
to slots and returns a new assembly.

Only labeled, exposed sides constrain matching -- a slot with `front_axis`
set but no `front_label` (not labeled), or `front_axis=None` (not exposed
there at all), imposes no requirement on that side. A slot can end up with
zero requirements (any leftover brick will do), one (e.g. a stretcher, whose
`back_axis` is always `None`), or two (e.g. a header, exposed -- and
possibly labeled differently -- on both faces).

Closure slots (`part.attributes["is_closure"]`, e.g. a half or quarter
brick -- see masonry.py) are dropped entirely -- not matched to a stock
brick, and not present in `result` at all (no placeholder, no copy of the
slot). They're a temporary stand-in for a real cut-brick workflow that
doesn't exist yet; excluding them is a known gap, not a final design, until
that's worked out.

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

Matching a brick to a slot
--------------------------
For any (slot, stock brick) pair, both possible orientations are scored by
summing, over the slot's requirement(s), how far that orientation's
side-score is from the label's target (a float in [0, 1] -- damage scores
are in that same range -- see `cellularized_part.py`); the better
orientation's cost is that pair's fit (see `_fit`). What differs between the
two `algorithm` choices is how slots and bricks get paired up using those
costs:

- `"hungarian"` (default): every constrained slot and every remaining stock
  brick's pairwise cost is assembled into one cost matrix, and
  `scipy.optimize.linear_sum_assignment` (the Hungarian algorithm) solves
  for the assignment that minimizes the TOTAL cost across all constrained
  slots at once -- the provably optimal matching, not just locally
  reasonable. Roughly O(n^3) in the number of constrained slots (n).
- `"greedy"`: slots are settled one at a time, most-constrained-first (two
  requirements before one), each taking whichever remaining brick fits it
  best *right now*. Cheaper (O(n x stock), no matrix, no scipy call), but
  with no lookahead an early slot can take a brick that would have fit a
  later slot better overall -- most visible with continuous
  `style="gradient"` labels (see label.py), where almost every slot wants a
  distinct target and there's little room for ties to paper over a bad
  greedy pick; less visible with discrete 0.0/1.0 labels, where many bricks
  tie for "good enough".

Leftover slots (no requirement at all) and leftover stock, after either
algorithm, are paired off in whatever order remains -- nothing to optimize
there.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.geometry import scale_vector
from scipy.optimize import linear_sum_assignment

from ..assembly import Assembly


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
        return sum(abs(sides[side] - label) for side, _axis, label in requirements)

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


def _match_greedy(constrained, remaining):
    """[(slot, brick, flipped, front_score, back_score), ...] via a greedy, one-slot-at-a-time heuristic.

    Slots are settled most-constrained-first (two requirements before one),
    each taking whichever remaining brick fits it best right now -- no
    lookahead across slots. See the module docstring for when this falls
    short of `_match_hungarian`.

    Parameters
    ----------
    constrained : list[(Part, list)]
        (slot, requirements) pairs, as built by :func:`allocate`.
    remaining : list[:class:`CellularizedPart`]
        Stock bricks available to assign. Not mutated -- a copy is consumed
        internally.

    Returns
    -------
    list[(Part, CellularizedPart, bool, float | None, float | None)]
    """
    # most-constrained (both faces labeled) first, so a brick that could
    # satisfy either a dual- or a single-sided slot goes to the harder one
    constrained = sorted(constrained, key=lambda pair: -len(pair[1]))
    remaining = list(remaining)

    assignments = []
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
        assignments.append(
            (
                slot,
                brick,
                best_flipped,
                front_score if "front" in sides else None,
                back_score if "back" in sides else None,
            )
        )

    return assignments, remaining


def _match_hungarian(constrained, remaining):
    """[(slot, brick, flipped, front_score, back_score), ...] via a global optimal assignment.

    Builds the full (constrained slot) x (remaining stock) cost matrix --
    each entry the better-orientation cost from :func:`_fit` -- and solves
    it with `scipy.optimize.linear_sum_assignment` (the Hungarian
    algorithm), which minimizes the TOTAL cost across every constrained slot
    at once, unlike `_match_greedy`'s one-slot-at-a-time heuristic. Each
    stock brick's end scores are computed once per axis and cached, since
    many slots share the same axis (e.g. every stretcher-exposed slot).

    Parameters
    ----------
    constrained : list[(Part, list)]
        (slot, requirements) pairs, as built by :func:`allocate`.
    remaining : list[:class:`CellularizedPart`]
        Stock bricks available to assign. Not mutated.

    Returns
    -------
    list[(Part, CellularizedPart, bool, float | None, float | None)]
    """
    if not constrained:
        return [], list(remaining)

    end_score_cache = {}

    def end_scores(index, axis):
        key = (index, axis)
        if key not in end_score_cache:
            end_score_cache[key] = _brick_end_scores(remaining[index], axis)
        return end_score_cache[key]

    cost_matrix = []
    flip_matrix = []
    for _slot, requirements in constrained:
        axis = requirements[0][1]
        cost_row, flip_row = [], []
        for index in range(len(remaining)):
            minus, plus = end_scores(index, axis)
            cost, flipped = _fit(requirements, minus, plus)
            cost_row.append(cost)
            flip_row.append(flipped)
        cost_matrix.append(cost_row)
        flip_matrix.append(flip_row)

    row_indices, col_indices = linear_sum_assignment(cost_matrix)

    assignments = []
    used = set()
    for row, col in zip(row_indices, col_indices):
        slot, requirements = constrained[row]
        axis = requirements[0][1]
        sides = set(side for side, _axis, _label in requirements)
        minus, plus = end_scores(col, axis)
        flipped = flip_matrix[row][col]
        front_score, back_score = (plus, minus) if flipped else (minus, plus)
        assignments.append(
            (
                slot,
                remaining[col],
                flipped,
                front_score if "front" in sides else None,
                back_score if "back" in sides else None,
            )
        )
        used.add(col)

    leftover = [brick for index, brick in enumerate(remaining) if index not in used]
    return assignments, leftover


_MATCHERS = {"greedy": _match_greedy, "hungarian": _match_hungarian}


def allocate(container, stock, algorithm="hungarian"):
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
    algorithm : {"hungarian", "greedy"}, optional
        Which strategy matches constrained slots (those with at least one
        labeled, exposed side) to stock bricks -- see the module docstring.
        "hungarian" (default): a global optimal assignment
        (`scipy.optimize.linear_sum_assignment`), minimizing total cost
        across all constrained slots at once. "greedy": a cheaper,
        one-slot-at-a-time heuristic with no lookahead. Unconstrained slots
        (no requirement at all) are unaffected either way -- always paired
        off with whatever stock is left over, in slot order. Closure slots
        (`is_closure`) never take part in this regardless of `algorithm` --
        see the module docstring.

    Returns
    -------
    :class:`assembly_information_model.Assembly`
        A new assembly with one part per container slot EXCEPT closures,
        which are dropped entirely (see the module docstring): a copy of the
        assigned stock brick, with its `frame` set to that slot's frame (or
        a 180-degree-flipped copy of it -- see `_flip_frame` -- if that
        orientation fit better), geometry left in the brick's own local
        coordinates, same convention as `build_wall`. Each part's key equals
        the SLOT's key in `container` (not the stock brick's own key -- see
        `source_key` below) -- so `container`'s connections (e.g. from
        `connect_wall`) carry straight over onto the result unchanged
        (except any connection touching a dropped closure, which is skipped
        instead), rather than needing `connect_wall` run again. Its
        `layer`/`course`/`position`/`front_axis`/`back_axis`/`front_label`/
        `back_label` attributes are copied over from the slot it was placed
        in, plus:
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
        If `stock` doesn't have enough parts for the container's non-closure
        slots, or `algorithm` isn't "hungarian" or "greedy".
    """
    if algorithm not in _MATCHERS:
        raise ValueError('algorithm must be "hungarian" or "greedy", got {!r}.'.format(algorithm))

    slots = sorted(
        container.parts(), key=lambda p: (p.attributes["layer"], p.attributes["course"], p.attributes["position"])
    )
    closures, regular = [], []
    for slot in slots:
        (closures if slot.attributes.get("is_closure") else regular).append(slot)

    stock_parts = list(stock.parts())
    if len(stock_parts) < len(regular):
        raise ValueError(
            "Not enough stock bricks ({}) for the container's {} non-closure slots.".format(
                len(stock_parts), len(regular)
            )
        )

    constrained, unconstrained = [], []
    for slot in regular:
        requirements = _slot_requirements(slot)
        (constrained if requirements else unconstrained).append((slot, requirements))

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

    # closures are dropped entirely -- not placed in `result` at all (see
    # the module docstring); `remaining` note below skips them too
    assignments, remaining = _MATCHERS[algorithm](constrained, stock_parts)
    for slot, brick, flipped, front_score, back_score in assignments:
        place(slot, brick, flipped, front_score, back_score)

    unconstrained.sort(
        key=lambda pair: (pair[0].attributes["layer"], pair[0].attributes["course"], pair[0].attributes["position"])
    )
    for (slot, _requirements), brick in zip(unconstrained, remaining):
        place(slot, brick, False, None, None)

    # container's own connections (if any -- e.g. from connect_wall) are
    # still valid as-is on result, since occupants share their slot's key --
    # except any connection touching a closure, which has no counterpart in
    # result at all now
    closure_keys = {slot.key for slot in closures}
    for (u, v), attrs in container.connections(data=True):
        if u in closure_keys or v in closure_keys:
            continue
        result.graph.add_edge(u, v, **attrs)

    return result


def summarize_allocation(result, bucket_size=0.1):
    """Print a per-(side, target) damage-score report for an assembly from `allocate`.

    A text alternative to eyeballing a colour-coded render: reports how many
    parts landed in each (front/back, target) bucket and the min/mean/max
    achieved damage score within it. Parts with no requirement on a given
    side (unlabeled or unexposed there) don't contribute to that side's
    buckets. Labels are floats in [0, 1] (see `label.py`) -- for a
    `style="gradient"` labeling, most parts carry a distinct value, so
    labels are rounded to the nearest `bucket_size` before grouping rather
    than bucketed by exact equality; for a discrete (banded) labeling, every
    label is already exactly 0.0 or 1.0, so this rounding is a no-op.

    Parameters
    ----------
    result : :class:`assembly_information_model.Assembly`
        Returned by :func:`allocate`.
    bucket_size : float, optional
        Rounding granularity for grouping labels, e.g. 0.1 groups into 11
        buckets (0.0, 0.1, ..., 1.0).
    """
    buckets = {}
    for part in result.parts():
        for side in ("front", "back"):
            label = part.attributes.get("{}_label".format(side))
            score = part.attributes.get("{}_damage_score".format(side))
            if label is None or score is None:
                continue
            bucket = round(round(label / bucket_size) * bucket_size, 10)
            buckets.setdefault((side, bucket), []).append(score)

    for (side, bucket), scores in sorted(buckets.items()):
        print(
            "{:>5} {:4.2f}: {:3d} parts | damage min={:.3f} mean={:.3f} max={:.3f}".format(
                side, bucket, len(scores), min(scores), sum(scores) / len(scores), max(scores)
            )
        )
