from compas.geometry import Frame, Vector, Box, Translation, Curve, Line, Point
from assembly_information_model import Assembly, Part, CellularizedPart, build_wall, connect_wall, label_facade
import os
import random

# dimensions full brick
L = 0.25
W = 0.12
H = 0.065

# create bricks
tool_frame = Frame([0, 0, 0], [1, 0, 0], [0, -1, 0])
brick_geometry = Box(L, W, H, Frame.worldXY(), 'box')
brick = Part.from_shape(brick_geometry, name="brick", frame=tool_frame)

# Container assembly generation.
compas_curve = Line(Point(0, 0, 0), Point(-3, 0, 0))
bond_type = 'flemish'
layers = 1
courses = 30

# Label parameters.
facade = 'front'
style = 'rows'
amount = 4
alternate = False

container_assembly = build_wall(brick, curve=compas_curve, bond_type=bond_type, num_layers=layers, num_courses=courses)
connect_wall(container_assembly)
label_facade(container_assembly, facade, style, amount, alternate)

print(container_assembly)

DATA = os.path.join(os.path.dirname(__file__), '..', 'data', 'allocation_data')
filename = "container_assembly.json"
PATH = os.path.join(DATA, filename)
container_assembly.to_json(PATH)