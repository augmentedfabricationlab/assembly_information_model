from compas.geometry import Frame, Vector, Box, Translation, Curve, Line, Point
from assembly_information_model import Assembly, Part, CellularizedPart, build_wall, connect_wall, label_facade, allocate
import os
import random

DATA = os.path.join(os.path.dirname(__file__), '..', 'data', 'allocation_data')

# Load container assembly and material stock from JSON files
filename = "container_assembly.json"
PATH = os.path.join(DATA, filename)
container_assembly = Assembly.from_json(PATH)

filename = "material_stock_simulated.json"
PATH = os.path.join(DATA, filename)
material_stock = Assembly.from_json(PATH)

# Allocate the container assembly using the material stock
allocated_assembly = allocate(container_assembly, material_stock)
print(allocated_assembly)

# Save the allocated assembly to a JSON file
filename = "allocated_assembly.json"
PATH = os.path.join(DATA, filename)
allocated_assembly.to_json(PATH)