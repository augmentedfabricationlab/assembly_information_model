
import os
import sys
import json

from compas.geometry import Translation

HERE = os.path.dirname(__file__)
DATA = os.path.abspath(os.path.join(HERE, "..", "data"))
ASSEMBLY_PATH = os.path.abspath(os.path.join(HERE, "..", "src"))
sys.path.append(ASSEMBLY_PATH)
PATH_TO = os.path.join(DATA, os.path.splitext(os.path.basename(__file__))[0] + ".json")
print(PATH_TO)

from assembly_information_model.assembly import Element, Assembly

# dimensions full brick
length = 0.240
width = 0.115
height = 0.050

brick = Element.from_dimensions(length, width, height, "full")
halfbrick = Element.from_dimensions(length/2, width, height, "half")

COURSES = 25
BRICKS_PER_COURSE = 4

MORTAR_PERPENDS = 0.025


total_length = BRICKS_PER_COURSE * length + (BRICKS_PER_COURSE - 1) * MORTAR_PERPENDS
gap_even = MORTAR_PERPENDS
gap_uneven = MORTAR_PERPENDS
gap_uneven = (total_length - (BRICKS_PER_COURSE * (length)))/BRICKS_PER_COURSE


assembly = Assembly() # initialize an empty assembly class

for row in range(COURSES):
    dy = row * height
    half_brick_ends = row % 2 != 0
    gap = gap_even if row % 2 == 0 else gap_uneven
    dx = 0

    bricks_in_course = BRICKS_PER_COURSE + (1 if half_brick_ends else 0)
    for j in range(bricks_in_course):

        first = j == 0
        last = j == bricks_in_course - 1

        is_half_brick = (first or last) and half_brick_ends

        if is_half_brick:
            if last:
                T = Translation([dx - width/2, 0, dy])
            else:
                T = Translation([dx - width/2-gap/2, 0, dy])
            e = halfbrick.transformed(T)
            dx += width+gap/2
        else:
            T = Translation([dx, 0, dy])
            e = brick.transformed(T)
            dx += length
        
        #R = Rotation.from_axis_and_angle([0,0,1], 0.1*j, point=e.gripping_frame.point) 
        #e.transform(R)
        assembly.add_element(e)
        dx += gap


assembly.transform(Translation([length/2, 0, 0]))

# 6. Save assembly to json
assembly.to_json(PATH_TO)
