"""
label.py
========

Labels one facade (front or back) of a `container` assembly (masonry.py)
with a "label" target in `[0, 1]` -- 1.0 meaning "fully bad" (target the
worst/most-damaged stock), 0.0 meaning "fully good" (target the
least-damaged stock), and anything in between a graded target -- via
`label_facade`. `style` picks one of four directional patterns
("rows"/"columns"/"checkered"/"cross"); with `gradient=False` (the default)
it produces discrete 0.0/1.0 bands/stripes/checkers, with `gradient=True` it
produces a continuous ramp along that same pattern instead. `build_wall`
itself has no opinion on any of this -- a slot's
course/position/layer/front_axis/back_axis says only where it sits and
which of its own faces are exposed, not what target it should be labeled
with. That's deliberately pulled out here so the same container can be
relabeled differently without rebuilding it.

Front and back are labeled independently: call `label_facade` once per
facade (each with its own `style`/`amount`) -- a slot exposed on both (e.g.
a header brick, whose `front_axis` and `back_axis` are both set) ends up
with its own independent "front_label" and "back_label".

See allocate.py for the next step -- assigning stock bricks to slots labeled
here, by matching each slot's label(s) against stock bricks' own damage
scores (also `[0, 1]` -- see cellularized_part.py).
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import math


def clean_labels(container):
    """Remove all labels from `container`."""
    for part in container.parts():
        part.attributes["front_label"] = None
        part.attributes["back_label"] = None


def _opposite(label):
    """1.0 - `label`, or `None` if `label` is `None`."""
    return None if label is None else 1.0 - label


def _row_col_stats(exposed_parts):
    """(lo, span, counts) shared by both `_band_function` and `_gradient_function`.

    `lo`/`span`: lowest `course` among `exposed_parts` and the number of
    distinct courses spanned. `counts`: {(layer, course): number of
    positions in that row} -- row length can vary (Flemish/English bond, or
    a curve), so column fractions/bands are computed *within* each row.
    """
    courses = [part.attributes["course"] for part in exposed_parts]
    lo, span = min(courses), max(courses) - min(courses) + 1
    counts = {}
    for part in exposed_parts:
        key = (part.attributes["layer"], part.attributes["course"])
        counts[key] = max(counts.get(key, 0), part.attributes["position"] + 1)
    return lo, span, counts


def _band_function(exposed_parts, style, amount):
    """A `Part -> int` band index, per `style`, for parts already known to be exposed."""
    lo, span, counts = _row_col_stats(exposed_parts)
    row_band = col_band = None

    if style in ("rows", "checkered"):

        def row_band(part):
            return min(int((part.attributes["course"] - lo) / span * amount), amount - 1)

    if style in ("columns", "checkered"):

        def col_band(part):
            key = (part.attributes["layer"], part.attributes["course"])
            return min(int(part.attributes["position"] / counts[key] * amount), amount - 1)

    if style == "rows":
        return row_band
    if style == "columns":
        return col_band
    if style == "checkered":
        return lambda part: row_band(part) + col_band(part)

    def cross_band(part):
        row_idx = part.attributes["course"] - lo
        key = (part.attributes["layer"], part.attributes["course"])
        col_idx = part.attributes["position"] / counts[key] * span
        on_cross = min(abs(row_idx - col_idx), abs(row_idx + col_idx - span)) < amount / 2
        return 0 if on_cross else 1

    return cross_band


def _sawtooth(fraction, amount):
    """Repeat `fraction` (in `[0, 1]`) into `amount` identical 0->1 ramps.

    `amount <= 1` just clamps `fraction` to `[0, 1]` -- one smooth ramp
    across the whole facade. `amount > 1` scales up and wraps (sawtooth),
    so e.g. `amount=2` gives two 0->1 ramps back to back -- except right at
    a ramp's own top edge (`fraction` exactly at a multiple of `1/amount`),
    which snaps back to 1.0 instead of wrapping to 0.0, so the highest
    course/position in each cycle reads as the ramp's actual peak rather
    than its reset.
    """
    scaled = min(max(fraction, 0.0), 1.0) * amount
    cycle = scaled % 1.0
    if cycle == 0.0 and scaled > 0.0:
        cycle = 1.0
    return cycle


def _gradient_function(exposed_parts, style, amount):
    """A `Part -> float` value in `[0, 1]`, per `style`, for parts already known to be exposed.

    The continuous analogue of `_band_function`: "rows"/"columns" ramp
    smoothly along the course/position direction instead of stepping
    through discrete bands; "checkered" ramps along the diagonal (the
    average of the row and column fractions); all three repeat `amount`
    times via `_sawtooth`. "cross" instead falls off continuously with
    distance from either diagonal of the facade -- 1.0 exactly on a
    diagonal, fading to 0.0 `amount` courses away -- `amount` here is a
    falloff width, not a repeat count.
    """
    lo, span, counts = _row_col_stats(exposed_parts)

    def row_fraction(part):
        return (part.attributes["course"] - lo) / (span - 1) if span > 1 else 0.0

    def col_fraction(part):
        key = (part.attributes["layer"], part.attributes["course"])
        n = counts[key]
        return part.attributes["position"] / (n - 1) if n > 1 else 0.0

    if style == "rows":
        return lambda part: _sawtooth(row_fraction(part), amount)
    if style == "columns":
        return lambda part: _sawtooth(col_fraction(part), amount)
    if style == "checkered":
        return lambda part: _sawtooth((row_fraction(part) + col_fraction(part)) / 2.0, amount)

    def cross_fraction(part):
        row_idx = part.attributes["course"] - lo
        key = (part.attributes["layer"], part.attributes["course"])
        col_idx = part.attributes["position"] / counts[key] * span
        dist = min(abs(row_idx - col_idx), abs(row_idx + col_idx - span))
        return max(0.0, 1.0 - dist / amount) if amount > 0 else (1.0 if dist == 0 else 0.0)

    return cross_fraction


def _bias(value, midpoint):
    """Remap `value` (in `[0, 1]`) through a power curve so input 0.5 maps to `midpoint`, while 0 and 1 stay fixed.

    The standard graphics "bias" function: `value ** (log(midpoint) /
    log(0.5))`. `midpoint=0.5` is the identity (today's plain linear ramp);
    pushing `midpoint` toward 1.0 bends the curve so values climb toward 1.0
    faster (most of the ramp's span reads "high"); toward 0.0 does the
    opposite. Never called with `midpoint` at exactly 0 or 1 -- see
    `label_facade`'s validation -- since either degenerates the curve flat
    except for a single-point jump at the other end.
    """
    if midpoint == 0.5:
        return value
    exponent = math.log(midpoint) / math.log(0.5)
    return value ** exponent


def _label_single_facade(container, facade, style, amount, alternate, gradient, midpoint):
    axis_key = "front_axis" if facade == "front" else "back_axis"
    label_key = "{}_label".format(facade)

    def exposed(part):
        # closure (e.g. half) bricks are excluded from labeling even where
        # they're otherwise exposed -- see masonry.py's `build_wall` -- a cut
        # brick isn't a meaningful grading target, so it's left with label
        # None like an unexposed slot
        return part.attributes[axis_key] is not None and not part.attributes.get("is_closure", False)

    exposed_parts = [part for part in container.parts() if exposed(part)]

    if not exposed_parts:
        for part in container.parts():
            part.attributes[label_key] = None
        return

    if gradient:
        value_of = _gradient_function(exposed_parts, style, amount)
        for part in container.parts():
            if exposed(part):
                value = _bias(value_of(part), midpoint)
                part.attributes[label_key] = value if alternate else 1.0 - value
            else:
                part.attributes[label_key] = None
        return

    band = _band_function(exposed_parts, style, amount)

    bad_band = 0 if alternate else 1
    for part in container.parts():
        if exposed(part):
            part.attributes[label_key] = 1.0 if band(part) % 2 == bad_band else 0.0
        else:
            part.attributes[label_key] = None


def label_facade(container, facade="front", style="rows", amount=2, alternate=True, gradient=False, midpoint=0.5):
    """Label one (or both) facade(s) of `container` with a `[0, 1]` target -- stripes, a checkerboard, or a gradient.

    Parameters
    ----------
    container : :class:`assembly_information_model.Assembly`
        Built by `build_wall`. Modified in place -- every call first clears
        *both* `front_label` and `back_label` (:func:`clean_labels`), so
        labeling front then back with two separate calls would wipe the
        first one out; use `facade="both"` for that instead.
    facade : {"front", "back", "both"}, optional
        Which exposed face(s) to label. "front"/"back": only slots whose
        `front_axis`/`back_axis` is not `None` get a label value -- every
        other slot's label is set to `None` instead, since that facade isn't
        exposed there at all. "both": front is labeled per
        `style`/`amount`/`alternate`/`gradient` same as `facade="front"`,
        and wherever `back_axis` is also exposed on that same slot,
        `back_label` is set to `1.0 - front_label` (e.g. a header slot
        labeled 1.0 on the front is 0.0 on the back) -- so front and back
        are never pulling toward the same target on the same slot.
    style : {"rows", "columns", "checkered", "cross"}, optional
        Which directional pattern to use -- discrete bands if
        `gradient=False` (default), a continuous ramp along the same
        pattern if `gradient=True` (see `gradient`). "rows": along
        consecutive `course`s. "columns": along consecutive `position`s,
        computed *within* each (layer, course) since course length can vary
        (Flemish/English bond, or a curve). "checkered": both row and
        column at once -- diagonal. "cross": from either diagonal of the
        facade (corner to corner); `position` is rescaled into course units
        (again *within* each (layer, course)) so the diagonals reach corner
        to corner even when row lengths vary.
    amount : float, optional
        If `gradient=False`: number of stripes (or, for `style="checkered"`,
        row/column bands per axis) for "rows"/"columns"/"checkered". Any
        positive integer: 1 means the whole facade gets one uniform label
        (all 1.0, or all 0.0 if `alternate` is False); even numbers split
        0.0/1.0 evenly; odd numbers (other than 1) leave one label with one
        extra band. For "cross": the cross's thickness, in courses (e.g.
        `amount=2` makes each diagonal arm of the X about 2 courses wide),
        independent of how many courses the facade spans.
        If `gradient=True`: for "rows"/"columns"/"checkered", the number of
        0->1 ramp cycles -- `amount=1` (not the default!) is one smooth
        gradient end to end; `amount=2` is two back-to-back ramps, etc. For
        "cross", instead the falloff width in courses -- how far from a
        diagonal the gradient reaches 0.0.
    alternate : bool, optional
        Which band/direction starts at 1.0 -- True (default): band 0 (e.g.
        bottom row / first column / bottom-left checker), or with
        `gradient=True`, the ramp's own starting end, is 1.0. False flips
        that. For `style="cross"` (banded or gradient), True makes the X
        itself (/its center) 1.0; False flips that. Only affects
        `facade="front"`/`"back"` directly; for `facade="both"`, back is
        always `1.0 - front_label` regardless of which end `alternate` made
        1.0 first.
    gradient : bool, optional
        False (default): discrete 0.0/1.0 bands, per `style` -- see
        `amount`. True: a continuous ramp across `[0, 1]` along the same
        `style` pattern instead of discrete bands.
    midpoint : float, optional
        Only used when `gradient=True` -- bends the ramp so its spatial
        midpoint (of each `amount` cycle) reads as `midpoint` instead of
        0.5, while the two ends of each cycle stay fixed at 0.0/1.0 (via a
        standard power-curve "bias" remap -- see `_bias`). Must be strictly
        between 0 and 1. E.g. `midpoint=0.8`: the middle of the facade (or
        of each cycle, if `amount>1`) is labeled 0.8 rather than 0.5, with
        values climbing toward 1.0 faster than they fall toward 0.0.
        `midpoint=0.5` (default) is a plain linear ramp, unchanged from
        before this parameter existed. With `alternate=False`, the
        midpoint's *labeled* value is `1.0 - midpoint` instead, since
        `alternate` flips the whole ramp after biasing.

    Returns
    -------
    None

    Raises
    ------
    ValueError
        If `amount` isn't a positive number, `style`/`facade` isn't one of
        the values above, or (`gradient=True` only) `midpoint` isn't
        strictly between 0 and 1.
    """
    if facade not in ("front", "back", "both"):
        raise ValueError('facade must be "front", "back", or "both", got {!r}.'.format(facade))
    if style not in ("rows", "columns", "checkered", "cross"):
        raise ValueError('style must be "rows", "columns", "checkered", or "cross", got {!r}.'.format(style))
    if amount <= 0:
        raise ValueError("amount must be a positive number, got {!r}.".format(amount))
    if gradient and not (0.0 < midpoint < 1.0):
        raise ValueError("midpoint must be strictly between 0 and 1, got {!r}.".format(midpoint))

    clean_labels(container)

    if facade != "both":
        _label_single_facade(container, facade, style, amount, alternate, gradient, midpoint)
        return

    _label_single_facade(container, "front", style, amount, alternate, gradient, midpoint)
    for part in container.parts():
        if part.attributes.get("back_axis") is None:
            continue
        part.attributes["back_label"] = _opposite(part.attributes.get("front_label"))
