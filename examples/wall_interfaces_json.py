"""Add a support plate to a wll assembly and identify the interfaces.

Steps
-----
1. Load an assembly from a json file
2. Compute the footprint of the assembly
3. Add a support in the XY plane at least the size to the footprint
4. Compute the interfaces of the assembly
5. Serialise the result

Parameters
----------
NMAX : int
    Maximum number of neighbors to be taken into account for the interface detection.
    Due to the shape of the support and the width of the wall, this number needs
    to be relatively high...
AMIN : float
    The minimum area of overlap between two faces for them to be considered to
    be in contact.

Exercise
--------
Change the values of ``NMAX`` and ``AMIN`` to understand their effect.

Notes
-----
Increasing ``NMAX`` is not necessary if the bottom blocks each have an individual support.

"""

import os
import sys
import json

from compas.geometry import Translation

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, "..", "data"))
PATH_FROM = os.path.join(DATA, 'stretcher_bond.json')

ASSEMBLY_PATH = os.path.abspath(os.path.join(HERE, "..", "src"))
sys.path.append(ASSEMBLY_PATH)

PATH_TO = os.path.join(DATA, os.path.splitext(os.path.basename(__file__))[0] + ".json")
print(PATH_TO)

from assembly_information_model.assembly import Assembly
from assembly_information_model.assembly.interfaces_numpy import assembly_interfaces_numpy_fromtojson

# parameters

NMAX = 100
AMIN = 0.0001

# load assembly from JSON
# assembly = Assembly.from_json(PATH_FROM)

# identify the interfaces
assembly_interfaces_numpy_fromtojson(PATH_FROM, PATH_TO, nmax=100, amin=0.0001)

# serialise
# assembly.to_json(PATH_TO)

assembly = Assembly.from_json(PATH_TO)

print(list(assembly.network.edges()))
