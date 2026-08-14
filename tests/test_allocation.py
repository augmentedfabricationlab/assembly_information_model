from compas.geometry import Frame, Vector, Box, Translation, Curve, Line, Point
from assembly_information_model import Assembly, Part, CellularizedPart, build_wall, connect_wall
import random

# dimensions full brick
L = 0.25
W = 0.12
H = 0.065

# create bricks
tool_frame = Frame([0, 0, 0], [1, 0, 0], [0, -1, 0])
brick_geometry = Box(L, W, H, Frame.worldXY(), 'box')
brick = CellularizedPart.from_shape(brick_geometry, grid=(4, 2, 2), name="brick", frame=tool_frame)

# Material stock generation.
courses = 20
rows = 5
columns = 10
gap_lateral = 0.035
gap_vertical = 0.02

material_stock = Assembly()

for course in range(courses):
    dz = (course * H) + (H / 2) #height of brick
    for row in range(rows):
        dx = - row * (L + gap_vertical) #row direction
        for column in range(columns):
            dy = column * (W + gap_lateral) #column direction
            T = Translation.from_vector([dx, dy, dz])
            brick_frame = Frame.from_transformation(T)
            part = brick.copy()
            part.frame = brick_frame
            part.set_damage_scores([random.random() for _ in range(part.num_cells)])
            material_stock.add_part(part, course=course, built=True)
            