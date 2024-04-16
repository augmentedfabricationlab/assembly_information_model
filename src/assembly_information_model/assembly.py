from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from copy import deepcopy

from compas.datastructures import Assembly as AssemblyCompas

class Assembly(AssemblyCompas):
    """A data structure for managing the connections between different parts of an assembly.

    Parameters
    ----------
    name : str, optional
        The name of the assembly.
    **kwargs : dict, optional
        Additional keyword arguments, which are stored in the attributes dict.

    Attributes
    ----------
    graph : :class:`compas.datastructures.Graph`
        The graph that is used under the hood to store the parts and their connections.

    See Also
    --------
    :class:`compas.datastructures.Graph`
    :class:`compas.datastructures.Mesh`
    :class:`compas.datastructures.VolMesh`

    """

    def __init__(self, name=None, **kwargs):
        super(Assembly, self).__init__()
    
    def __str__(self):
        tpl = "This assembly contains {} parts and {} connections."
        return tpl.format(self.graph.number_of_nodes(), self.graph.number_of_edges())

    def transform(self, T):
        """Transforms this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None
        """
        for part in self.parts():
            part.transform(T)

    def transformed(self, T):
        """Returns a transformed copy of this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Assembly
        """
        assembly = self.copy()
        assembly.transform(T)
        return assembly
    
    def copy(self):
        """Returns a copy of this assembly.
        """
        cls = type(self)
        return cls.__from_data__(deepcopy(self.__data__))

    def part(self, key):
        """Get a part by its key."""

        return self.graph.node[key]['part']

