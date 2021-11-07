"""Identify the courses of a assembly assembly.

Steps
-----
1. Load an assembly from a JSON file.
2. Identify the course rows
3. serialise to JSON

Notes
-----
This will only work as expected on *wall* assemblies that are properly supported.

Exercise
--------
Check if all blocks of bottom course are supported.

"""
import os
import sys

from compas.geometry import Frame, Box, Transformation, Translation, Vector

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, "..", "data"))
ASSEMBLY_PATH = os.path.abspath(os.path.join(HERE, "..", "src"))
sys.path.append(ASSEMBLY_PATH)

PATH_FROM = os.path.join(DATA, 'assembly_collision_check.json')

from assembly_information_model.assembly import Assembly, Element
from assembly_information_model.assembly.collision import collision_check_cgal

from compas_cgal.intersections import intersection_mesh_mesh
from compas_cgal.meshing import remesh

print("loading the assembly: ")
# load an assembly from JSON
assembly = Assembly.from_json(PATH_FROM)

#element'S dimensions
width = 0.024
length = 0.4
height = 0.048
a = 0.048

# element
box_frame = Frame([0, 0, height/2.], [1, 0, 0], [0, 1, 0]) #center of the box frame
box = Box(box_frame, length, width, height)
tool_frame =  Frame([0, 0, height], [1, 0, 0], [0, 1, 0])
conn_frame_1 = Frame([length-a, width, 0], [1, 0, 0], [0, 1, 0])
conn_frame_2 = Frame([-length+a, -width, 0], [1, 0, 0], [0, 1, 0])

elem_x = Element.from_shape(box, box_frame)

elem_x.tool_frame = tool_frame
elem_x.connector_frame_1 = conn_frame_1
elem_x.connector_frame_2 = conn_frame_2
elem_x._type = 'X'
elem_x._base_frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])

T = Translation.from_vector(Vector(0.2, 0 , 0))
new_elem = elem_x.transformed(T)

collided_keys = collision_check_cgal(assembly, new_elem)

print(collided_keys)
