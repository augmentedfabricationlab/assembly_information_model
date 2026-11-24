# How brick allocation works

`allocate(container, stock, ...)` places graded stock bricks into a labeled
wall. This note explains, in plain words, what happens, and the maths behind
the `face_p` option.

## The steps

**1. Each slot gets a target.**
The labeling (`label.label_facade`) gives every visible slot face a number
from 0 to 1 saying how damaged the brick showing there should look:
0 = clean, 1 = as broken as possible, anything in between with a gradient.
Slots that are hidden or unlabeled get no target and take no brick from
stock.

**2. Each brick's faces get one damage number.**
Every face of a scanned brick is a small grid of cells, each with its own
damage score. Those cells are combined into one number for the face
(`CellularizedPart.face_damage`):

- `face_p=1` (default): the plain average. One deep break surrounded by
  clean cells gets averaged away.
- `face_p=4`: the worst cells count much more. A single deep break makes the
  whole face read as damaged, which matches how the face is judged by eye.

Only the exposed face is scored. A stretcher slot looks at the brick's long
face, a header slot at its end. The top and bottom faces are never scored.

**3. A brick can be turned around.**
Each brick has two opposite faces it could show, for example its two long
sides on a stretcher. The algorithm turns it 180° so the better-matching face
shows. For a header that is labeled on both sides of the wall, both ends must
fit at once.

**4. Every brick gets a cost for every slot.**
The cost is the gap between the brick's face score and the slot's target,
squared:

$$\text{cost} = (\text{score} - \text{target})^2$$

A small gap is cheap and a big gap is expensive. Squaring keeps a gradient in
order: the worse brick always goes where the target is higher. With the
unsquared gap, two bricks that both fall short of their targets cost the same
either way round, and the order is left to chance.

**5. The best overall arrangement is chosen.**
The Hungarian algorithm (`algorithm="hungarian"`, the default) looks at all
bricks and all slots together and picks the arrangement with the lowest total
cost. It's a global best fit, not first come, first served. Within a run of
slots that all have the same target (0/1 rows), which brick goes where is
arbitrary, because every swap costs the same.

**6. Optional: edge-aware (`edge_aware=True`, off by default).**
Before scoring, it spreads damage around each vertical corner from one face to
the next. In testing it pushed scores around in ways that didn't match the
real bricks, so it is left off.

**In one sentence:** each brick face is boiled down to one damage number
(weighing its worst spots when `face_p` > 1), and then the solver places the
bricks so that, across the whole wall, each visible face is as close as
possible to how damaged that spot is meant to look.

## The maths of `face_p`

### The formula

The face score is the *power mean* of the face's cell scores
$s_1, \dots, s_n$:

$$\text{face score} = \left( \frac{s_1^p + s_2^p + \dots + s_n^p}{n} \right)^{1/p}$$

It has three steps:

1. **Raise** each cell's damage score to the power `p`.
2. **Average** those results.
3. **Take the p-th root** to get back to the original 0–1 scale.

With `p = 1`, steps 1 and 3 do nothing, so it's the plain average.

### Why the worst cells dominate at p = 4

Damage scores are between 0 and 1, and raising such a number to the 4th
power shrinks it, but small numbers shrink far more than big ones:

| cell score | to the 4th power |
|---|---|
| 0.45 | 0.041 |
| 0.10 | 0.0001 |
| 0.03 | 0.0000008 |

After step 1, the clean cells are essentially zero and the bad cell is all
that's left. The root in step 3 then brings that value back to a readable
size.

### Worked example: brick 33's bad face

Cells: `0.45, 0.03, 0.03, 0.02, 0, 0, 0, 0` (8 cells)

**p = 1 (plain average):**

$$\frac{0.45 + 0.03 + 0.03 + 0.02}{8} = 0.066$$

The break is spread across all 8 cells and almost disappears.

**p = 4:**

1. To the 4th power: 0.041, ≈0, ≈0, ≈0, 0, 0, 0, 0
2. Average: 0.041 / 8 = 0.0051
3. 4th root: $0.0051^{1/4} = 0.266$

For comparison, brick 32's worse face has damage spread over more cells
(`0.27, 0.11, 0.09, 0.07, 0.06, 0.06, 0.05, 0.00`). It scores higher than 33
with the plain average (0.089 vs 0.066) but lower at p = 4 (0.163 vs 0.266),
which matches how the two faces look.

### The shortcut

If one cell is bad and the rest are clean, the face score is:

$$\text{worst cell} \times \left(\tfrac{1}{n}\right)^{1/p}$$

With 8 cells:

- **p = 1:** the bad cell keeps 1/8 = **12.5%** of its value (0.45 → 0.056)
- **p = 4:** it keeps $(1/8)^{1/4}$ = **59%** of its value (0.45 → 0.27)
- **p very large:** it keeps **100%**, so the score is just the worst cell
  (the max)

So `p` is one control for how much a single deep break counts against the
whole face: 1 is a fair average, 4 means the worst spot mostly decides, and
higher values approach the max.

## Usage

```python
allocated = allocate(container, stock, edge_aware=False, face_p=4)
```

The `front_damage_score` / `back_damage_score` stored on each placed brick are
computed with the `face_p` used, so scores from runs with different `face_p`
values aren't directly comparable.
