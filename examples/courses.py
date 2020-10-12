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

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, "..", "data"))
ASSEMBLY_PATH = os.path.abspath(os.path.join(HERE, "..", "src"))
sys.path.append(ASSEMBLY_PATH)

PATH_FROM = os.path.join(DATA, 'stretcher_bond.json')
PATH_TO = os.path.join(DATA, 'stretcher_bond_courses.json')

from assembly_information_model.assembly import Assembly
from assembly_information_model.assembly.courses import assembly_courses, assembly_with_interfaces_courses

print("loading the assembly: ")
# load an assembly from JSON
assembly = Assembly.from_json(PATH_FROM)

# identify the courses
#assembly_with_interfaces_courses(assembly) #if the interfaces are already calculated
assembly_courses(assembly) #without calculated interfaces

# example test: get the top keys
c_max = max(assembly.network.nodes_attribute('course'))
print(c_max)
keys_on_top = list(assembly.network.nodes_where({'course': c_max}))

print(keys_on_top)

# serialise the result
assembly.to_json(PATH_TO)
