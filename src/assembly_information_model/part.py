from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.datastructures import Datastructure

from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron


class Part(Datastructure):
    """A data structure for representing assembly parts.

    Parameters
    ----------
    name : str, optional
        The name of the part.
        The name will be stored in :attr:`Part.attributes`.
    frame : :class:`compas.geometry.Frame`, optional
        The local coordinate system of the part.

    Attributes
    ----------
    attributes : dict[str, Any]
        General data structure attributes that will be included in the data dict and serialization.
    key : int or str
        The identifier of the part in the connectivity graph of the parent assembly.
    frame : :class:`compas.geometry.Frame`
        The local coordinate system of the part.
    features : list(:class:`compas.datastructures.Feature`)
        The features added to the base shape of the part's geometry.

    """

    DATASCHEMA = {
        "type": "object",
        "properties": {
            "attributes": {"type": "object"},
            "key": {"type": ["integer", "string"]},
            "frame": Frame.DATASCHEMA,
        },
        "required": ["key", "frame"],
    }

    @property
    def __data__(self):
        return {
            "attributes": self.attributes,
            "key": self.key,
            "frame": self.frame.__data__,
        }

    @classmethod
    def __from_data__(cls, data):
        part = cls()
        part.attributes.update(data["attributes"] or {})
        part.key = data["key"]
        part.frame = Frame.__from_data__(data["frame"])
        return part

    def __init__(self, name=None, frame=None, **kwargs):
        super(Part, self).__init__()
        self.attributes = {"name": name or "Part"}
        self.attributes.update(kwargs)
        self.key = None
        self.frame = frame or Frame.worldXY()
    
    @classmethod
    def from_mesh(cls, mesh, name=None, frame=None):
        """Construct a part from a mesh.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure describing the part.
        frame : :class:`Frame`
            Origin frame of the part.
        name : str
            Name of the part.

        Returns
        -------
        :class:`Part`
            New instance of part.
        """
        part = cls(name, frame)
        part.mesh = mesh

        return part

    @classmethod
    def from_shape(cls, shape, name=None, frame=None):
        """Construct an element from a shape primitive.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
            Shape primitive describing the part.
        frame : :class:`Frame`
            Origin frame of the part.
        name : str
            Name of the part.

        Returns
        -------
        :class:`Part`
            New instance of part.
        """
        part = cls(name, frame) 
        mesh = Mesh.from_shape(shape)

        part.mesh = mesh
        part.shape = shape

        return part

    @property
    def frame(self):
        """Get frame of the element."""
        return self._frame

    @frame.setter
    def frame(self, frame):
        """Set frame of the part."""
        self._frame = frame.copy()

    @property
    def centroid(self):
        mesh = self.attributes['mesh']
        return mesh.centroid()

    @property
    def face_frames(self):
        """Compute the local frame of each face of the part's mesh.

        Returns
        -------
        dict
            A dictionary mapping face identifiers to face frames.
        """
        mesh = self.attributes['mesh']
        return {fkey: self.face_frame(fkey) for fkey in mesh.faces()}

    def face_frame(self, fkey):
        """Compute the frame of a specific face of the part's mesh.

        Parameters
        ----------
        fkey : hashable
            The identifier of the frame.

        Returns
        -------
        frame
            The frame of the specified face.
        """
        mesh = self.attributes['mesh']

        xyz = mesh.face_coordinates(fkey)
        o = mesh.face_center(fkey)
        w = mesh.face_normal(fkey)
        u = [xyz[1][i] - xyz[0][i] for i in range(3)]  # align with longest edge instead?
        v = cross_vectors(w, u)
        uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
        return o, uvw

    @property
    def top(self):
        """Identify the *top* face of the part's mesh.

        Returns
        -------
        int
            The identifier of the face.

        Notes
        -----
        The face with the highest centroid is considered the *top* face.
        """
        mesh = self.attributes['mesh']

        fkey_centroid = {fkey: mesh.face_center(fkey) for fkey in mesh.faces()}
        fkey, _ = sorted(fkey_centroid.items(), key=lambda x: x[1][2])[-1]
        return fkey

    @property
    def center(self):
        """Compute the center of mass of the part's mesh..

        Returns
        -------
        point
            The center of mass of the part's mesh..
        """
        mesh = self.attributes['mesh']

        vertices = [mesh.vertex_coordinates(key) for key in mesh.vertices()]
        faces = [mesh.face_vertices(fkey) for fkey in mesh.faces()]
        return centroid_polyhedron((vertices, faces))

    @property
    def volume(self):
        """Compute the volume of the part's mesh.

        Returns
        -------
        float
            The volume of the part's mesh.
        """
        mesh = self.attributes['mesh']
        
        vertices = [mesh.vertex_coordinates(key) for key in mesh.vertices()]
        faces = [mesh.face_vertices(fkey) for fkey in mesh.faces()]
        v = volume_polyhedron((vertices, faces))
        return v

    @property
    def mesh(self):
        """Returns the mesh of the part, if available.

        Returns
        -------
        :class:`Mesh`
        """

        if 'mesh' in self.attributes.keys():
            return self.attributes['mesh']
    
    @mesh.setter
    def mesh(self, mesh):
        """Sets the mesh of the part, if available.

        Parameters
        ----------
        :class:`Mesh`
        """
        self.attributes.update({'mesh':mesh})

    @property
    def shape(self):
        """Returns the shape of the part, if available.

        Returns
        -------
        :class:`Shape`
        """

        if 'shape' in self.attributes.keys():
            return self.attributes['shape']
    
    @shape.setter
    def shape(self, shape):
        """Sets the shape of the part, if available.

        Parameters
        ----------
        :class:`Shape`
        """
        self.attributes.update({'shape':shape})
    def transform(self, T):
        """Transforms the element.

        Parameters
        ----------
        T : :class:`Transformation`

        Returns
        -------
        None

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
        >>> part.transform(Translation.from_vector([1, 0, 0]))
        """
        self.frame.transform(T)

        if 'mesh' in self.attributes.keys():
            self.attributes['mesh'].transform(T)

        if 'shape' in self.attributes.keys():
            self.attributes['shape'].transform(T)
        

    def transformed(self, T):
        """Returns a transformed copy of this part.

        Parameters
        ----------
        T : :class:`Transformation`

        Returns
        -------
        Part

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
        >>> part2 = part.transformed(Translation.from_vector([1, 0, 0]))
        """
        part = self.copy()
        part.transform(T)
        return part

    def copy(self):
        """Returns a copy of this part.

        Returns
        -------
        Part
        """
        part = Part(name=self.attributes['name'], frame=self.frame.copy())
        part.key = self.key
        
        if 'mesh' in self.attributes.keys():
            part.attributes.update({'mesh':self.attributes['mesh'].copy()})
        if 'shape' in self.attributes.keys():
            part.attributes.update({'shape':self.attributes['shape'].copy()})

        return part
