"""
label.py
========

Labels one facade (front or back) of a `container` assembly (masonry.py)
with alternating "bad"/"good" stripes or a checkerboard, via `label_facade`.
`build_wall` itself has no opinion on this at all -- a slot's course/
position/layer/front_axis/back_axis says only where it sits and which of its
own faces are exposed, not whether any of that is "bad" or "good". That's
deliberately pulled out here so the same container can be relabeled
differently without rebuilding it.

Front and back are labeled independently: call `label_facade` once per
facade (each with its own `style`/`amount`) -- a slot exposed on both (e.g.
a header brick, whose `front_axis` and `back_axis` are both set) ends up
with its own independent "front_label" and "back_label".

See allocate.py for the next step -- assigning stock bricks to slots labeled
here.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division


def clean_labels(container):
    """Remove all "bad"/"good" labels from `container`."""
    for part in container.parts():
        part.attributes["front_label"] = None
        part.attributes["back_label"] = None


_OPPOSITE = {"bad": "good", "good": "bad"}


def _band_function(exposed_parts, style, amount):
    """A `Part -> int` band index, per `style`, for parts already known to be exposed."""
    row_band = col_band = None

    if style in ("rows", "checkered"):
        courses = [part.attributes["course"] for part in exposed_parts]
        lo, span = min(courses), max(courses) - min(courses) + 1

        def row_band(part):
            return min(int((part.attributes["course"] - lo) / span * amount), amount - 1)

    if style in ("columns", "checkered"):
        counts = {}
        for part in exposed_parts:
            key = (part.attributes["layer"], part.attributes["course"])
            counts[key] = max(counts.get(key, 0), part.attributes["position"] + 1)

        def col_band(part):
            key = (part.attributes["layer"], part.attributes["course"])
            return min(int(part.attributes["position"] / counts[key] * amount), amount - 1)

    if style == "rows":
        return row_band
    if style == "columns":
        return col_band
    return lambda part: row_band(part) + col_band(part)


def _label_single_facade(container, facade, style, amount, alternate):
    axis_key = "front_axis" if facade == "front" else "back_axis"
    label_key = "{}_label".format(facade)

    def exposed(part):
        return part.attributes[axis_key] is not None

    exposed_parts = [part for part in container.parts() if exposed(part)]

    if not exposed_parts:
        for part in container.parts():
            part.attributes[label_key] = None
        return

    band = _band_function(exposed_parts, style, amount)

    bad_band = 0 if alternate else 1
    for part in container.parts():
        if exposed(part):
            part.attributes[label_key] = "bad" if band(part) % 2 == bad_band else "good"
        else:
            part.attributes[label_key] = None


def label_facade(container, facade="front", style="rows", amount=2, alternate=True):
    """Label one (or both) facade(s) of `container` with alternating "bad"/"good" stripes or a checkerboard.

    Parameters
    ----------
    container : :class:`assembly_information_model.Assembly`
        Built by `build_wall`. Modified in place -- every call first clears
        *both* `front_label` and `back_label` (:func:`clean_labels`), so
        labeling front then back with two separate calls would wipe the
        first one out; use `facade="both"` for that instead.
    facade : {"front", "back", "both"}, optional
        Which exposed face(s) to label. "front"/"back": only slots whose
        `front_axis`/`back_axis` is not `None` get a "bad"/"good" value --
        every other slot's label is set to `None` instead, since that facade
        isn't exposed there at all. "both": front is labeled per
        `style`/`amount`/`alternate` same as `facade="front"`, and wherever
        `back_axis` is also exposed on that same slot, `back_label` is set
        to the exact opposite of `front_label` (e.g. a header slot labeled
        "good" on the front is "bad" on the back, and vice versa) -- so
        front and back are never both "bad"/both "good" on the same slot.
    style : {"rows", "columns", "checkered"}, optional
        "rows": horizontal stripes -- bands of consecutive `course`s.
        "columns": vertical stripes -- bands of consecutive `position`s,
        computed *within* each (layer, course) since course length can vary
        (Flemish/English bond, or a curve). "checkered": both at once -- a
        slot's row band and column band (same `amount` for each) are summed,
        and "bad"/"good" alternate on *that* combined parity, so `amount`
        controls how large each checker square is (higher = finer).
    amount : int, optional
        Number of stripes (or, for `style="checkered"`, row/column bands per
        axis). Any positive integer: 1 means the whole facade gets one
        uniform label (all "bad", or all "good" if `alternate` is False);
        even numbers split "bad"/"good" evenly; odd numbers (other than 1)
        leave one label with one extra band.
    alternate : bool, optional
        Which band starts "bad" -- True (default): band 0 (e.g. bottom row /
        first column / bottom-left checker) is "bad". False: flips that, so
        band 0 is "good" instead. Only affects `facade="front"`/`"back"`
        directly; for `facade="both"`, back is always front's opposite
        regardless of which one `alternate` made "bad" first.

    Returns
    -------
    None

    Raises
    ------
    ValueError
        If `amount` isn't a positive integer, or `style`/`facade` isn't one
        of the values above.
    """
    if facade not in ("front", "back", "both"):
        raise ValueError('facade must be "front", "back", or "both", got {!r}.'.format(facade))
    if style not in ("rows", "columns", "checkered"):
        raise ValueError('style must be "rows", "columns", or "checkered", got {!r}.'.format(style))
    if amount < 1:
        raise ValueError("amount must be a positive integer, got {!r}.".format(amount))

    clean_labels(container)

    if facade != "both":
        _label_single_facade(container, facade, style, amount, alternate)
        return

    _label_single_facade(container, "front", style, amount, alternate)
    for part in container.parts():
        if part.attributes.get("back_axis") is None:
            continue
        front_label = part.attributes.get("front_label")
        part.attributes["back_label"] = _OPPOSITE[front_label] if front_label is not None else None
