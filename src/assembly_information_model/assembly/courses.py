from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


__all__ = ['assembly_courses']


def assembly_with_interfaces_courses(assembly):
    """Identify the courses in a wall of bricks.

    Parameters
    ----------
    wall : Assembly
        The wall assembly data structure.

    Examples
    --------
    .. code-block:: python

        pass

    """
    courses = []

    # all element keys
    elements = set(assembly.network.nodes())

    # base course keys
    c_min = min(assembly.network.nodes_attribute(name='z'))
    #keys_on_bottom = list(assembly.network.nodes_where({'z': c_min}))
    base = set(assembly.network.nodes_where({'z': c_min}))

    if base:
        courses.append(list(base))

        seen = set()
        seen.update(base)

        elements -= base

        while elements:

            nbrs = set(nbr for key in courses[-1] for nbr in assembly.network.node_neighbors(key))
            #print(nbrs)
            course = list(nbrs - seen)
            courses.append(course)
            seen.update(nbrs)
            elements -= nbrs

    # assign course id's to the corresponding blocks
    for i, course in enumerate(courses):
        assembly.network.nodes_attribute(name='course', value=i, keys=course)
    #return courses

def assembly_courses(assembly, tol=0.001):
    """Identify the courses in a wall of bricks.

    Parameters
    ----------
    wall : Assembly
        The wall assembly data structure.

    Examples
    --------
    .. code-block:: python

        pass

    """
    courses = []

    # all element keys
    elements = set(assembly.network.nodes())

    # base course keys
    c_min = min(assembly.network.nodes_attribute('z'))
    #base = set(assembly.network.nodes_where({'z': c_min}))

    base = set()
    for e in elements:
        z = assembly.network.node_attribute(key=e, name='z')
        if (z - c_min) ** 2 < tol:
            base.add(e)
    #print(base)

    if base:
        courses.append(list(base))

        elements -= base

        while elements: #and counter<1000:

            c_min = min([assembly.network.node_attribute(key=key, name='z') for key in elements])
            #print(c_min)
            #base = set(assembly.network.nodes_where({'z': c_min}))
            #print(base)
            base = set()
            for e in elements:
                z = assembly.network.node_attribute(key=e, name='z')
                if (z - c_min) ** 2 < tol:
                    base.add(e)
            
            courses.append(list(base))
            elements -= base

    # assign course id's to the corresponding blocks
    for i, course in enumerate(courses):
        assembly.network.nodes_attribute(name='course', value=i, keys=course)

# ==============================================================================
# Main
# ==============================================================================

if __name__ == '__main__':
    pass
