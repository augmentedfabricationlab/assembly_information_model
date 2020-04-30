from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh
from compas.datastructures import mesh_transform

from compas.geometry import Frame
from compas.geometry import Box
from compas.geometry import centroid_points
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron

from .utilities import _deserialize_from_data
from .utilities import _serialize_to_data


__all__ = ['Element']


class Element(object):
    """Data structure representing a discrete elements of an assembly.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame of the element.

    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(Element, self).__init__()
        self.frame = frame
        self.id = ""
        self.trajectory = None
        self._gripping_frame = None
        self._source = None
        self._mesh = None

    @classmethod
    def from_mesh(cls, mesh, frame):
        """Construct an element from a mesh.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure.
        frame : :class:`Frame`
            Origin frame of the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        element = cls(frame)
        element._source = element._mesh = mesh
        return element

    @classmethod
    def from_shape(cls, shape, frame):
        """Construct an element from a shape primitive.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
            Shape primitive describing the element.
        frame : :class:`Frame`
            Origin frame of the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        element = cls(frame)
        element._source = shape
        element._mesh = Mesh.from_shape(element._source)
        return element

    @classmethod
    def from_box(cls, box):
        """Construct an element from a box primitive.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Box primitive describing the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        return cls.from_shape(box, box.frame)

    @classmethod
    def from_dimensions(cls, length=240, width=115, height=50, typology="full", gripping_frame=None):
        """Construct an element with a box primitive with the given dimensions.

        Parameters
        ----------
        length : float
            length of the brick.
        width : float
            width of the brick.
        height : float
            height of the brick.
        typology : "string"
            type of the brick, "full" or "half"
        gripping_frame : :class:`Frame`
            frame for robot gripping.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        if not gripping_frame:
            gripping_frame = Frame([0, 0, height], [1, 0, 0], [0, -1, 0]) #frame for robot gripping
        
        box_frame = Frame([-length/2., -width/2., 0.], [1, 0, 0], [0, 1, 0]) #center of the box frame
        box_frame = Frame([0., 0., height/2], [1, 0, 0], [0, 1, 0]) #center of the box frame
        box = Box(box_frame, length, width, height)
        
        element = cls(gripping_frame) # element initialisation
        element.id = typology
        element._source = box
        element._mesh = Mesh.from_shape(element._source)

        return element

    @classmethod
    def from_polysurface(cls, guid):
        """Class method for constructing a block from a Rhino poly-surface.

        Parameters
        ----------
        guid : str
            The GUID of the poly-surface.

        Returns
        -------
        Block
            The block corresponding to the poly-surface.

        Notes
        -----
        In Rhino, poly-surfaces are organised such that the cycle directions of
        the individual sub-surfaces produce normal vectors that point out of the
        enclosed volume. The normal vectors of the faces of the mesh, therefore
        also point "out" of the enclosed volume.
        """
        from compas_rhino.geometry import RhinoSurface
        surface = RhinoSurface.from_guid(guid)
        return surface.brep_to_compas(cls)

    @classmethod
    def from_rhinomesh(cls, guid):
        """Class method for constructing a block from a Rhino mesh.

        Parameters
        ----------
        guid : str
            The GUID of the mesh.

        Returns
        -------
        Block
            The block corresponding to the Rhino mesh.
        """
        from compas_rhino.geometry import RhinoMesh
        mesh = RhinoMesh.from_guid(guid)
        return mesh.to_compas(cls)

    @property
    def mesh(self):
        """Mesh of the element."""
        if not self._source:
            return None

        if self._mesh:
            return self._mesh

        if isinstance(self._source, Mesh):
            self._mesh = self._source
        else:
            self._mesh = Mesh.from_shape(self._source)

        return self._mesh

    @mesh.setter
    def mesh(self, mesh):
        self._source = self._mesh = mesh

    @property
    def frame(self):
        """Frame of the element."""
        return self._frame

    @frame.setter
    def frame(self, frame):
        self._frame = frame.copy()

    @property
    def gripping_frame(self):
        """Gripping frame of the element."""
        if not self._gripping_frame:
            self._gripping_frame = self.frame.copy()

        return self._gripping_frame

    @gripping_frame.setter
    def gripping_frame(self, frame):
        self._gripping_frame = frame.copy() if frame else None
    
    @property
    def pose_quaternion(self):
        """ formats the element's gripping frame to a pose quaternion and returns the pose"""
        return list(self._gripping_frame.point) + list(self._gripping_frame.quaternion)

    @property
    def centroid(self):
        return self.mesh.centroid()

    @property
    def face_frames(self):
        """Compute the local frame of each face of the element's mesh.

        Returns
        -------
        dict
            A dictionary mapping face identifiers to face frames.
        """
        return {fkey: self.face_frame(fkey) for fkey in self._mesh.faces()}

    def face_frame(self, fkey):
        """Compute the frame of a specific face.

        Parameters
        ----------
        fkey : hashable
            The identifier of the frame.

        Returns
        -------
        frame
            The frame of the specified face.
        """
        xyz = self._mesh.face_coordinates(fkey)
        o = self._mesh.face_center(fkey)
        w = self._mesh.face_normal(fkey)
        u = [xyz[1][i] - xyz[0][i] for i in range(3)]  # align with longest edge instead?
        v = cross_vectors(w, u)
        uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
        return o, uvw

    @property
    def top(self):
        """Identify the *top* face of the element's mesh.

        Returns
        -------
        int
            The identifier of the face.

        Notes
        -----
        The face with the highest centroid is considered the *top* face.
        """
        frames = self.face_frames()
        fkey_centroid = {fkey: self._mesh.face_center(fkey) for fkey in self._mesh.faces()}
        fkey, _ = sorted(fkey_centroid.items(), key=lambda x: x[1][2])[-1]
        return fkey

    @property
    def center(self):
        """Compute the center of mass of the block.

        Returns
        -------
        point
            The center of mass of the block.
        """
        vertices = [self._mesh.vertex_coordinates(key) for key in self._mesh.vertices()]
        faces = [self._mesh.face_vertices(fkey) for fkey in self._mesh.faces()]
        return centroid_polyhedron((vertices, faces))

    @property
    def volume(self):
        """Compute the volume of the block.

        Returns
        -------
        float
            The volume of the block.
        """
        vertices = [self.vertex_coordinates(key) for key in self.vertices()]
        faces = [self.face_vertices(fkey) for fkey in self.faces()]
        v = volume_polyhedron((vertices, faces))
        return v

    @classmethod
    def from_data(cls, data):
        """Construct an element from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Element
            The constructed element.
        """
        element = cls(Frame.worldXY())
        element.data = data
        return element

    @property
    def data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> element = Element(Frame.worldXY())
        >>> print(element.data)
        """
        d = dict(frame=self.frame.to_data())

        # Only include gripping plane if attribute is really set
        # (unlike the property getter that defaults to `self.frame`)
        if self._gripping_frame:
            d['gripping_frame'] = self.gripping_frame.to_data()

        if self._source:
            d['_source'] = _serialize_to_data(self._source)
        
        if self._mesh:
            d['_mesh'] = _serialize_to_data(self._mesh)

        # Probably best to store JointTrajectory instead of JointTrajectoryPoints
        if self.trajectory:
            d['trajectory'] = [p.to_data() for p in self.trajectory]

        if self.id:
            d['id'] = self.id
            
        return d

    @data.setter
    def data(self, data):
        self.frame = Frame.from_data(data['frame'])
        if 'gripping_frame' in data:
            self.gripping_frame = Frame.from_data(data['gripping_frame'])
        if '_source' in data:
            self._source = _deserialize_from_data(data['_source'])
        if '_mesh' in data:
            self._mesh = _deserialize_from_data(data['_mesh']) 
        if 'trajectory' in data:
            #self.trajectory = [JointTrajectoryPoint.from_data(d) for d in data['trajectory']]
            #self.trajectory = _deserialize_from_data(data['trajectory']) 
            self.trajectory = [Frame.from_data(d) for d in data['trajectory']]
        if 'id' in data:
            self.id = data['id']

    def to_data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> e1 = Element(Frame.worldXY())
        >>> e2 = Element.from_data(element.to_data())
        >>> e2.frame == Frame.worldXY()
        True
        """
        return self.data

    def transform(self, transformation):
        """Transforms the element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element.transform(Translation([1, 0, 0]))
        """
        self.frame.transform(transformation)
        if self._gripping_frame:
            self.gripping_frame.transform(transformation)
        if self._source:
            if type(self._source) == Mesh:
                mesh_transform(self._source, transformation)  # it would be really good to have Mesh.transform()
            else:
                self._source.transform(transformation)
        if self._mesh:
            mesh_transform(self._mesh, transformation)  # it would be really good to have Mesh.transform()
    
    def transformed(self, transformation):
        """Returns a transformed copy of this element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Element

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element2 = element.transformed(Translation([1, 0, 0]))
        """
        elem = self.copy()
        elem.transform(transformation)
        return elem

    def copy(self):
        """Returns a copy of this element.

        Returns
        -------
        Element
        """
        elem = Element(self.frame.copy())
        if self._gripping_frame:
            elem.gripping_frame = self.gripping_frame.copy()
        if self._source:
            elem._source = self._source.copy()
        if self._mesh:
            elem._mesh = self._mesh.copy()
        if self.id:
            elem.id = self.id
        return elem
