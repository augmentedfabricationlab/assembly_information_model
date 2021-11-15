from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import os
from copy import deepcopy
from compas.datastructures import Network, network

from .element import Element

from .utilities import FromToData
from .utilities import FromToJson

__all__ = ['Assembly']


class Assembly(FromToData, FromToJson):
    """A data structure for discrete element assemblies.

    An assembly is essentially a network of assembly elements.
    Each element is represented by a node of the network.
    Each interface or connection between elements is represented by an edge of the network.

    Attributes
    ----------
    network : :class:`compas.Network`, optional
    elements : list of :class:`Element`, optional
        A list of assembly elements.
    attributes : dict, optional
        User-defined attributes of the assembly.
        Built-in attributes are:
        * name (str) : ``'Assembly'``
    default_element_attribute : dict, optional
        User-defined default attributes of the elements of the assembly.
        The built-in attributes are:
        * is_planned (bool) : ``False``
        * is_placed (bool) : ``False``
    default_connection_attributes : dict, optional
        User-defined default attributes of the connections of the assembly.

    Examples
    --------
    >>> assembly = Assembly()
    >>> for i in range(2):
    >>>     element = Element.from_box(Box(Frame.worldXY(), 10, 5, 2))
    >>>     assembly.add_element(element)
    """

    def __init__(self,
                 elements=None,
                 attributes=None,
                 default_element_attributes=None,
                 default_connection_attributes=None):

        self.network = Network()
        self.network.attributes.update({'name': 'Assembly'})

        if attributes is not None:
            self.network.attributes.update(attributes)

        self.network.default_node_attributes.update({
            'is_planned': False,
            'is_placed': False
        })

        if default_element_attributes is not None:
            self.network.default_node_attributes.update(default_element_attributes)

        if default_connection_attributes is not None:
            self.network.default_edge_attributes.update(default_connection_attributes)

        if elements:
            for element in elements:
                self.add_element(element)

    @property
    def name(self):
        """str : The name of the assembly."""
        return self.network.attributes.get('name', None)

    @name.setter
    def name(self, value):
        self.network.attributes['name'] = value

    def number_of_elements(self):
        """Compute the number of elements of the assembly.

        Returns
        -------
        int
            The number of elements.

        """
        return self.network.number_of_nodes()

    def number_of_connections(self):
        """Compute the number of connections of the assembly.

        Returns
        -------
        int
            the number of connections.

        """
        return self.network.number_of_edges()

    @property
    def data(self):
        """Return a data dictionary of the assembly.
        """
        # Network data does not recursively serialize to data...
        d = self.network.data
        # so we need to trigger that for elements stored in nodes
        node = {}
        for vkey, vdata in d['node'].items():
            node[vkey] = {key: vdata[key] for key in vdata.keys() if key != 'element'}
            node[vkey]['element'] = vdata['element'].to_data()
        d['node'] = node
        return d

    @data.setter
    def data(self, data):
        # Deserialize elements from node dictionary
        for _vkey, vdata in data['node'].items():
            vdata['element'] = Element.from_data(vdata['element'])

        self.network = Network.from_data(data)

    def clear(self):
        """Clear all the assembly data."""
        self.network.clear()

    def add_element(self, element, key=None, attr_dict={}, **kwattr):
        """Add an element to the assembly.

        Parameters
        ----------
        element : Element
            The element to add.
        attr_dict : dict, optional
            A dictionary of element attributes. Default is ``None``.

        Returns
        -------
        hashable
            The identifier of the element.
        """
        attr_dict.update(kwattr)
        x, y, z = element.frame.point
        key = self.network.add_node(key=key, attr_dict=attr_dict,
                                    x=x, y=y, z=z, element=element)
        return key

    def add_connection(self, u, v, attr_dict=None, **kwattr):
        """Add a connection between two elements and specify its attributes.

        Parameters
        ----------
        u : hashable
            The identifier of the first element of the connection.
        v : hashable
            The identifier of the second element of the connection.
        attr_dict : dict, optional
            A dictionary of connection attributes.
        kwattr
            Other connection attributes as additional keyword arguments.

        Returns
        -------
        tuple
            The identifiers of the elements.
        """
        return self.network.add_edge(u, v, attr_dict, **kwattr)

    def transform(self, transformation):
        """Transforms this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None
        """
        for _k, element in self.elements(data=False):
            element.transform(transformation)

    def transformed(self, transformation):
        """Returns a transformed copy of this assembly.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Assembly
        """
        assembly = self.copy()
        assembly.transform(transformation)
        assembly.network.transform(transformation)
        return assembly

    def copy(self):
        """Returns a copy of this assembly.
        """
        cls = type(self)
        return cls.from_data(deepcopy(self.data))

    def element(self, key, data=False):
        """Get an element by its key."""
        if data:
            return self.network.node[key]['element'], self.network.node[key]
        else:
            return self.network.node[key]['element']

    def elements(self, data=False):
        """Iterate over the elements of the assembly.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next element as a (key, element) tuple, if ``data`` is ``False``.
        3-tuple
            The next element as a (key, element, attr) tuple, if ``data`` is ``True``.

        """
        if data:
            for vkey, vattr in self.network.nodes(True):
                yield vkey, vattr['element'], vattr
        else:
            for vkey in self.network.nodes(data):
                yield vkey, self.network.node[vkey]['element']

    def connections(self, data=False):
        """Iterate over the connections of the network.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next connection identifier (u, v), if ``data`` is ``False``.
        3-tuple
            The next connection as a (u, v, attr) tuple, if ``data`` is ``True``.

        """
        return self.network.edges(data)

    def export_to_json_for_xr(self, path, is_built=False):

        self.network.update_default_node_attributes({"is_built":False,"idx_v":None,"custom_attr_1":None,"custom_attr_2":None,"custom_attr_3":None})

        for key, element in self.elements():
            idx_v = self.network.node_attribute(key, "course")
            self.network.node_attribute(key, "idx_v", idx_v)
            self.network.node_attribute(key, "is_built", is_built)

        self.to_json(path)
