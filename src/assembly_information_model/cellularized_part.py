"""
CellularizedPart
=================

Extends :class:`Part` so that, alongside its geometry (mesh/shape/frame), a
Part can also carry a per-cell grid -- e.g. for damage grading.

The grid is stored on a :class:`PartCellNetwork` (a
:class:`compas.datastructures.CellNetwork` subclass) attached to the part as
``part.cell_network``. It starts out empty and is filled in by
:meth:`PartCellNetwork.build_cell_grid`, which subdivides a reference box into
an (nx, ny, nz) grid of hexahedral cells (default 2x2x2 = 8 octants; use e.g.
(4, 2, 2) or (2, 2, 4) for 16 cells along whichever axis you're splitting
further). Each cell carries a `damage_score` attribute (float in [0, 1], or
None/NaN if ungraded) plus a `bin` label like "000" (matching its (i, j, k)
grid index), so scores from an external grader can be dropped in directly as
long as its cell ordering is mapped to the same (i, j, k) convention.

- If the part's `shape` is an exact :class:`compas.geometry.Box` (e.g. a
  brick), the grid subdivides that box directly and every cell is `active`
  (the box is fully solid within its own bounding box).
- Otherwise (an arbitrary mesh, or a non-box shape), the grid subdivides the
  *bounding box* of the part's mesh, and each cell is tested against the
  actual mesh and marked `active=False` if it falls outside. By default
  (``box_mode=True``) this is a cheap voxelization -- containment of the
  cell's centroid only, every cell stays a plain grid box, active or not. With
  ``box_mode=False``, each cell is instead clipped (pure-compas half-space
  clipping, no boolean-mesh dependency) against the part's actual mesh, and
  the exact irregular result is stored as that cell's `cell_mesh` attribute
  -- the real geometry subdivided into cells, not a box approximation.

This is intentionally decoupled from ROS/torch/BlenderProc/etc -- it only
depends on compas, so it can be unit tested standalone.
"""

from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import itertools
import math

from compas.datastructures import CellNetwork
from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import intersection_line_triangle
from compas.geometry import subtract_vectors
from compas.geometry import add_vectors
from compas.geometry import scale_vector
from compas.geometry import dot_vectors
from compas.geometry import normalize_vector

from .part import Part


# A fixed, non-axis-aligned direction for the containment ray-cast: reduces
# the chance of the ray grazing an edge or vertex of an axis-aligned mesh.
_RAY_DIRECTION = normalize_vector([0.5257311121, 0.6881909602, 0.4999999998])


def _point_in_mesh(point, mesh, direction=None, tol=1e-9):
    """Ray-casting point-in-mesh test (parity of forward intersections).

    Works for arbitrary closed meshes, including concave/non-convex ones.
    Faces are fan-triangulated from their first vertex, so non-convex faces
    are only handled correctly if they're star-shaped from that vertex.
    Like any ray-casting containment test, points exactly on the mesh
    boundary, or rays that exactly graze an edge/vertex, are an ill-defined
    edge case.

    Parameters
    ----------
    point : [float, float, float]
    mesh : :class:`compas.datastructures.Mesh`
    direction : [float, float, float], optional
    tol : float, optional

    Returns
    -------
    bool
    """
    direction = direction or _RAY_DIRECTION
    far = add_vectors(point, scale_vector(direction, 1e6))
    line = (point, far)

    count = 0
    for fkey in mesh.faces():
        verts = mesh.face_coordinates(fkey)
        for i in range(1, len(verts) - 1):
            triangle = (verts[0], verts[i], verts[i + 1])
            x = intersection_line_triangle(line, triangle, tol=tol)
            if x is None:
                continue
            t = dot_vectors(subtract_vectors(x, point), direction)
            if t > tol:
                count += 1
    return count % 2 == 1


def _clip_polygon_halfspace(polygon, plane_point, plane_normal, tol=1e-9):
    """Sutherland-Hodgman clip of a single planar polygon against a half-space.

    Keeps the side where ``dot(plane_normal, p - plane_point) <= tol``, i.e.
    `plane_normal` is the *outward* direction of the half-space being cut away.

    Parameters
    ----------
    polygon : list[[float, float, float]]
        Ordered polygon vertices (assumed planar).
    plane_point : [float, float, float]
    plane_normal : [float, float, float]
    tol : float, optional

    Returns
    -------
    list[[float, float, float]]
        The clipped polygon, possibly empty (fully outside) or unchanged
        (fully inside).
    """
    n = len(polygon)
    if n == 0:
        return []

    dists = [dot_vectors(subtract_vectors(p, plane_point), plane_normal) for p in polygon]
    result = []
    for i in range(n):
        cur_pt, cur_d = polygon[i], dists[i]
        nxt_pt, nxt_d = polygon[(i + 1) % n], dists[(i + 1) % n]
        cur_inside = cur_d <= tol
        if cur_inside:
            result.append(cur_pt)
        if cur_inside != (nxt_d <= tol):
            t = cur_d / (cur_d - nxt_d)
            result.append(add_vectors(cur_pt, scale_vector(subtract_vectors(nxt_pt, cur_pt), t)))
    return result


def _clip_mesh_by_plane(mesh, plane_point, plane_normal, tol=1e-9, precision=9):
    """Clip a closed mesh against a single half-space, capping the cut.

    Every face is clipped independently (via :func:`_clip_polygon_halfspace`),
    cut vertices are welded by rounded coordinate, and the resulting open
    boundary loop(s) -- the intersection of the mesh surface with the cutting
    plane -- are capped with new planar faces so the result stays closed.

    A single-plane cut always leaves a boundary that lies entirely on that
    one plane, so each loop is safely planar and can be filled with one new
    face. That stops being true once several planes have cut through the same
    corner at once (their combined boundary is no longer planar) -- which is
    why cutting by more than one plane must go one plane at a time, capping
    in between, rather than clipping every plane first and capping once.

    Parameters
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        Must be closed (watertight) for the cap to be well-defined.
    plane_point : [float, float, float]
    plane_normal : [float, float, float]
        Outward direction of the half-space being cut away.
    tol : float, optional
    precision : int, optional
        Decimal rounding used to weld coincident cut vertices.

    Returns
    -------
    :class:`compas.datastructures.Mesh`
        A new, possibly empty (no faces left), closed mesh.
    """
    clipped = Mesh()
    vertex_map = {}

    def get_vertex(point):
        key = tuple(round(c, precision) for c in point)
        if key not in vertex_map:
            vertex_map[key] = clipped.add_vertex(x=point[0], y=point[1], z=point[2])
        return vertex_map[key]

    for fkey in mesh.faces():
        polygon = _clip_polygon_halfspace(mesh.face_coordinates(fkey), plane_point, plane_normal, tol=tol)
        if len(polygon) < 3:
            continue
        face_vertices = [get_vertex(p) for p in polygon]
        face_vertices = [v for i, v in enumerate(face_vertices) if v != face_vertices[i - 1]]
        if len(face_vertices) >= 3:
            clipped.add_face(face_vertices)

    for loop in clipped.vertices_on_boundaries():
        # `vertices_on_boundaries` returns each loop closed (first vertex
        # repeated at the end); drop the repeat and use the winding as-is --
        # it's already the correct orientation to fill the hole.
        loop = loop[:-1] if len(loop) > 1 and loop[0] == loop[-1] else loop
        if len(loop) >= 3:
            clipped.add_face(loop)

    return clipped


def _slice_mesh(mesh, origin, axis, offsets, tol=1e-9):
    """Split a closed mesh into ``len(offsets) + 1`` closed slabs along one axis.

    Slices incrementally (clip off the lowest slab, keep clipping what
    *remains*) rather than re-clipping the full mesh against every offset
    independently -- each original face only ever gets processed by the
    handful of slabs it actually straddles, not by every slab in the grid.
    That's what makes slicing a whole nx*ny*nz grid cost roughly one pass
    over the mesh rather than O(cells x mesh faces).

    Parameters
    ----------
    mesh : :class:`compas.datastructures.Mesh`
    origin : [float, float, float]
        A point with local coordinate 0 along `axis` (e.g. a box's frame
        origin -- must be consistent with how `offsets` were measured).
    axis : [float, float, float]
        Unit vector along which to slice; slabs are ordered low-to-high.
    offsets : list[float]
        Sorted interior cut distances from `origin` along `axis` (the grid's
        interior breakpoints for this axis -- not the two outer bounds).
    tol : float, optional

    Returns
    -------
    list[:class:`compas.datastructures.Mesh`]
        ``len(offsets) + 1`` closed slabs, low to high along `axis`.
    """
    slabs = []
    remaining = mesh
    for offset in offsets:
        point = add_vectors(origin, scale_vector(axis, offset))
        low = _clip_mesh_by_plane(remaining, point, axis, tol=tol)
        remaining = _clip_mesh_by_plane(remaining, point, scale_vector(axis, -1), tol=tol)
        slabs.append(low)
    slabs.append(remaining)
    return slabs


class PartCellNetwork(CellNetwork):
    """A :class:`compas.datastructures.CellNetwork` used to store a
    :class:`CellularizedPart`'s per-cell grid.

    Instances start empty -- call :meth:`build_cell_grid` to subdivide a
    reference box into an (nx, ny, nz) grid of cells.
    """

    @classmethod
    def __from_data__(cls, data):
        net = super(PartCellNetwork, cls).__from_data__(data)
        net._reindex()
        return net

    def _reindex(self):
        self.ckey_to_ijk = {ckey: tuple(self.cell_attribute(ckey, "ijk")) for ckey in self.cells()}
        self.ijk_to_ckey = {ijk: ckey for ckey, ijk in self.ckey_to_ijk.items()}

    def cell_at(self, i, j, k):
        """The cell key at grid index (i, j, k)."""
        return self.ijk_to_ckey[(i, j, k)]

    def active_cells(self):
        """Cell keys flagged `active` (part of the part's actual shape)."""
        return [ckey for ckey in self.cells() if self.cell_attribute(ckey, "active")]

    def cell_mesh(self, cell):
        """The cell's clipped irregular mesh, if built with ``box_mode=False``.

        Returns
        -------
        :class:`compas.datastructures.Mesh` | None
        """
        return self.cell_attribute(cell, "cell_mesh")

    def cell_box(self, cell):
        """The cell's axis-aligned box, straight from its 8 grid-vertex points.

        A much cheaper stand-in for :meth:`cell_to_mesh`/:meth:`cell_mesh`
        when a caller only needs each cell's extent -- e.g. a lightweight
        box preview instead of a full mesh -- since every grid cell (built
        with ``box_mode=True``) already is a box; this skips reconstructing
        face/vertex mesh topology for it. Not meaningful for a
        ``box_mode=False`` cell's actual clipped shape -- use
        :meth:`cell_mesh` for that instead.

        Returns
        -------
        :class:`compas.geometry.Box`
        """
        points = self.cell_points(cell)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        center = [(min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0, (min(zs) + max(zs)) / 2.0]
        dx, dy, dz = max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)
        return Box(dx, dy, dz, frame=Frame(center, [1, 0, 0], [0, 1, 0]))

    def build_cell_grid(self, box, grid=(2, 2, 2), mesh=None, tol=1e-6, box_mode=True):
        """Fill this (empty) network with a regular grid of hexahedral
        cells subdividing `box`, in `box`'s own frame.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Reference box whose volume is subdivided into the grid. For a
            box-shaped part (e.g. a brick) this is the part's own exact
            shape. For an irregular part, pass its bounding box instead
            (e.g. ``part.mesh.aabb()``).
        grid : tuple[int, int, int], optional
            Number of subdivisions along the box's local (x, y, z) axes,
            e.g. (2, 2, 2) -> 8 cells, (4, 2, 2) / (2, 2, 4) -> 16 cells.
        mesh : :class:`compas.datastructures.Mesh`, optional
            If given, each cell is tested against this mesh. Use this for
            irregular / non-box shapes so the active cells approximate the
            mesh's actual (possibly irregular) volume rather than its plain
            bounding box.
        tol : float, optional
            Tolerance forwarded to the containment ray-cast / clipping.
        box_mode : bool, optional
            True (default): a cell is `active` if its centroid falls inside
            `mesh` (or always active if `mesh` is None) -- a cheap
            voxelization where every cell is a plain grid box, active or not.
            False: additionally clip `mesh` against each cell's own 6
            half-spaces (pure-compas, no boolean-mesh dependency) and store
            the exact result as the cell's `cell_mesh` attribute -- the
            actual part geometry subdivided into irregular cells, rather than
            a box approximation. A cell is `active` if that clip is
            non-empty. Slower, and only meaningful when `mesh` is given.

        Returns
        -------
        :class:`PartCellNetwork`
            self
        """
        self.clear()

        nx, ny, nz = grid
        L, W, H = box.xsize, box.ysize, box.zsize

        xs = [-L / 2 + L * i / nx for i in range(nx + 1)]
        ys = [-W / 2 + W * j / ny for j in range(ny + 1)]
        zs = [-H / 2 + H * k / nz for k in range(nz + 1)]

        vids = {}
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for k, z in enumerate(zs):
                    wx, wy, wz = box.frame.to_world_coordinates([x, y, z])
                    vids[(i, j, k)] = self.add_vertex(x=wx, y=wy, z=wz)

        # slice the whole grid up front (hierarchically: x-slabs, each split
        # into y-rows, each split into z-cells) instead of re-clipping the
        # full mesh against every individual cell from scratch -- each face
        # only gets processed by the slabs it actually straddles this way.
        cell_meshes = None
        if not box_mode and mesh is not None:
            cell_meshes = {}
            x_slabs = _slice_mesh(mesh, box.frame.point, box.frame.xaxis, xs[1:-1], tol=tol)
            for i, x_slab in enumerate(x_slabs):
                y_slabs = _slice_mesh(x_slab, box.frame.point, box.frame.yaxis, ys[1:-1], tol=tol)
                for j, y_slab in enumerate(y_slabs):
                    z_slabs = _slice_mesh(y_slab, box.frame.point, box.frame.zaxis, zs[1:-1], tol=tol)
                    for k, z_slab in enumerate(z_slabs):
                        cell_meshes[(i, j, k)] = z_slab

        for i, j, k in itertools.product(range(nx), range(ny), range(nz)):
            v000 = vids[(i, j, k)]
            v100 = vids[(i + 1, j, k)]
            v010 = vids[(i, j + 1, k)]
            v110 = vids[(i + 1, j + 1, k)]
            v001 = vids[(i, j, k + 1)]
            v101 = vids[(i + 1, j, k + 1)]
            v011 = vids[(i, j + 1, k + 1)]
            v111 = vids[(i + 1, j + 1, k + 1)]

            face_vertices = [
                [v000, v100, v110, v010],  # bottom (z-)
                [v001, v101, v111, v011],  # top (z+)
                [v000, v100, v101, v001],  # front (y-)
                [v010, v110, v111, v011],  # back (y+)
                [v000, v010, v011, v001],  # left (x-)
                [v100, v110, v111, v101],  # right (x+)
            ]
            faces = [self.add_face(fv) for fv in face_vertices]
            ckey = self.add_cell(faces)

            ijk = (i, j, k)
            self.cell_attribute(ckey, "ijk", ijk)
            self.cell_attribute(ckey, "bin", "".join(str(v) for v in ijk))
            self.cell_attribute(ckey, "damage_score", None)

            if not box_mode and mesh is not None:
                cell_mesh = cell_meshes[(i, j, k)]
                self.cell_attribute(ckey, "cell_mesh", cell_mesh)
                active = cell_mesh.number_of_faces() > 0
            elif mesh is not None:
                active = _point_in_mesh(self.cell_centroid(ckey), mesh, tol=tol)
            else:
                active = True
            self.cell_attribute(ckey, "active", active)

        self._reindex()
        return self


class CellularizedPart(Part):
    """A Part with an attached :class:`PartCellNetwork` cell grid.

    Pass a `shape` (e.g. a :class:`compas.geometry.Box` for a brick) and/or a `mesh`. If `shape` is
    an exact `Box`, the grid subdivides it directly. Otherwise, the grid
    subdivides the bounding box of `mesh` and cells outside the actual mesh
    are deactivated -- see :meth:`PartCellNetwork.build_cell_grid`.

    Parameters
    ----------
    shape : :class:`compas.geometry.Shape`, optional
        Exact shape of the part, e.g. a `Box` for a brick.
    mesh : :class:`compas.datastructures.Mesh`, optional
        Mesh of the part. Required if `shape` is not given (irregular
        parts); derived from `shape` automatically otherwise.
    grid : tuple[int, int, int], optional
        Number of subdivisions along (x, y, z). Default (2, 2, 2) -> 8
        cells.
    box_mode : bool, optional
        True (default): cells are plain grid boxes, flagged active/inactive
        by containment. False: cells are clipped to the part's actual mesh
        geometry -- see :meth:`PartCellNetwork.build_cell_grid`.
    name : str, optional
    frame : :class:`compas.geometry.Frame`, optional
    """

    def __init__(self, shape=None, mesh=None, grid=(2, 2, 2), box_mode=True, name=None, frame=None, **kwargs):
        super(CellularizedPart, self).__init__(name=name, frame=frame, **kwargs)

        if shape is not None:
            self.shape = shape
            self.mesh = mesh if mesh is not None else Mesh.from_shape(shape)
        elif mesh is not None:
            self.mesh = mesh
        else:
            raise ValueError("CellularizedPart needs a `shape` and/or a `mesh` to build its cell grid from.")

        self.attributes["grid"] = tuple(grid)
        self.attributes["box_mode"] = box_mode
        self.cell_network = PartCellNetwork()
        self.build_cell_grid(grid=grid, box_mode=box_mode)

    @classmethod
    def from_box(cls, size, grid=(2, 2, 2), box_mode=True, name=None, frame=None, **kwargs):
        """Convenience constructor for a box-shaped part, e.g. a brick.

        Parameters
        ----------
        size : tuple[float, float, float]
            Box (length, width, height).
        grid : tuple[int, int, int], optional
        box_mode : bool, optional
        name : str, optional
        frame : :class:`compas.geometry.Frame`, optional

        Returns
        -------
        :class:`CellularizedPart`
        """
        frame = frame or Frame.worldXY()
        box = Box(size[0], size[1], size[2], frame=frame.copy())
        return cls(shape=box, grid=grid, box_mode=box_mode, name=name, frame=frame, **kwargs)

    @classmethod
    def from_shape(cls, shape, grid=(2, 2, 2), box_mode=True, name=None, frame=None, **kwargs):
        """Construct a cellularized part from a shape primitive.

        Overrides :meth:`Part.from_shape`, whose `cls(name, frame)` positional
        call assumes the base `Part.__init__(name, frame)` signature and would
        misassign `name`/`frame` into this class's `shape`/`mesh` parameters.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
        grid : tuple[int, int, int], optional
        box_mode : bool, optional
        name : str, optional
        frame : :class:`compas.geometry.Frame`, optional

        Returns
        -------
        :class:`CellularizedPart`
        """
        return cls(shape=shape, grid=grid, box_mode=box_mode, name=name, frame=frame, **kwargs)

    @classmethod
    def from_mesh(cls, mesh, grid=(2, 2, 2), box_mode=True, name=None, frame=None, **kwargs):
        """Construct a cellularized part from a mesh.

        See :meth:`from_shape` for why this overrides the `Part` base version.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
        grid : tuple[int, int, int], optional
        box_mode : bool, optional
        name : str, optional
        frame : :class:`compas.geometry.Frame`, optional

        Returns
        -------
        :class:`CellularizedPart`
        """
        return cls(mesh=mesh, grid=grid, box_mode=box_mode, name=name, frame=frame, **kwargs)

    # ------------------------------------------------------------------
    # Cell grid
    # ------------------------------------------------------------------

    def build_cell_grid(self, grid=None, box_mode=None):
        """(Re)build `self.cell_network` from the part's current shape/mesh.

        Parameters
        ----------
        grid : tuple[int, int, int], optional
            Defaults to the grid the part was last built with.
        box_mode : bool, optional
            Defaults to the mode the part was last built with. See
            :meth:`PartCellNetwork.build_cell_grid`.
        """
        grid = tuple(grid) if grid is not None else self.attributes["grid"]
        self.attributes["grid"] = grid
        box_mode = self.attributes.get("box_mode", True) if box_mode is None else box_mode
        self.attributes["box_mode"] = box_mode

        if isinstance(self.shape, Box) and box_mode:
            box = self.shape
            containment_mesh = None
        else:
            # non-box mode always clips against `self.mesh` -- it exists even
            # for exact Box shapes (derived automatically in __init__).
            box = self.mesh.aabb()
            containment_mesh = self.mesh

        self.cell_network.build_cell_grid(box, grid=grid, mesh=containment_mesh, box_mode=box_mode)

    @property
    def num_cells(self):
        return self.cell_network.number_of_cells()

    # ------------------------------------------------------------------
    # Damage grid access
    # ------------------------------------------------------------------

    def set_damage_scores(self, scores):
        """Set damage scores on the grid.

        Parameters
        ----------
        scores : dict[str, float] | dict[tuple[int,int,int], float] | list[float]
            Either a mapping from bin label ("000", "010", ...) or (i, j, k)
            tuple to a damage score, or a flat list in the same (i, j, k)
            iteration order used at construction time (matches
            ``itertools.product(range(nx), range(ny), range(nz))``).
        """
        net = self.cell_network
        if isinstance(scores, dict):
            for key, score in scores.items():
                if isinstance(key, str):
                    ijk = tuple(int(c) for c in key)
                else:
                    ijk = tuple(key)
                ckey = net.ijk_to_ckey[ijk]
                net.cell_attribute(ckey, "damage_score", score)
        else:
            # flat list, same order cells were created in
            for ckey, score in zip(net.cells(), scores):
                net.cell_attribute(ckey, "damage_score", score)

    def get_damage_scores(self, active_only=True):
        """Return {bin_label: damage_score} for all (active) cells.

        Parameters
        ----------
        active_only : bool, optional
            If True (default), only include cells flagged `active` -- i.e.
            skip cells that were voxelized away for an irregular shape.
        """
        net = self.cell_network
        return {
            net.cell_attribute(ckey, "bin"): net.cell_attribute(ckey, "damage_score")
            for ckey in net.cells()
            if not active_only or net.cell_attribute(ckey, "active")
        }

    def mean_damage(self):
        """Mean damage score across all graded (non-None/NaN) active cells."""
        scores = [
            s
            for s in self.get_damage_scores().values()
            if s is not None and not (isinstance(s, float) and math.isnan(s))
        ]
        if not scores:
            return None
        return sum(scores) / len(scores)

    def face_damage(self, axis, side):
        """Mean damage score of the cells on one face of the grid.

        Parameters
        ----------
        axis : {'x', 'y', 'z'}
            Which axis is normal to the face of interest.
        side : {'-', '+'}
            The low (-) or high (+) side along that axis.

        Returns
        -------
        float | None
            Mean damage score of the (active) cells touching that face, or
            None if none of them are graded.
        """
        nx, ny, nz = self.attributes["grid"]
        axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
        n_along = (nx, ny, nz)[axis_idx]
        target = 0 if side == "-" else n_along - 1

        net = self.cell_network
        scores = []
        for ckey in net.cells():
            if not net.cell_attribute(ckey, "active"):
                continue
            ijk = net.cell_attribute(ckey, "ijk")
            if ijk[axis_idx] == target:
                s = net.cell_attribute(ckey, "damage_score")
                if s is not None and not (isinstance(s, float) and math.isnan(s)):
                    scores.append(s)
        if not scores:
            return None
        return sum(scores) / len(scores)

    # ------------------------------------------------------------------
    # Serialization: extend Part's __data__ with shape/mesh + cell grid
    # ------------------------------------------------------------------

    @property
    def __data__(self):
        # Only Box shapes round-trip exactly; other shapes (Sphere, Cylinder,
        # arbitrary meshes, ...) are persisted as mesh only, same as parts
        # built directly from a mesh.
        return {
            "attributes": {k: v for k, v in self.attributes.items() if k not in ("mesh", "shape")},
            "key": self.key,
            "frame": self.frame.__data__,
            "mesh": self.mesh.__data__ if self.mesh is not None else None,
            "shape": self.shape.__data__ if isinstance(self.shape, Box) else None,
            "cell_network": self.cell_network.__data__,
        }

    @classmethod
    def __from_data__(cls, data):
        frame = Frame.__from_data__(data["frame"])
        mesh = Mesh.__from_data__(data["mesh"]) if data.get("mesh") else None
        shape = Box.__from_data__(data["shape"]) if data.get("shape") else None

        part = cls(
            shape=shape,
            mesh=mesh if shape is None else None,
            grid=data["attributes"]["grid"],
            frame=frame,
        )
        part.attributes.update(data["attributes"])
        part.key = data["key"]
        part.cell_network = PartCellNetwork.__from_data__(data["cell_network"])
        return part

    def copy(self):
        part = CellularizedPart(
            shape=self.shape.copy() if self.shape is not None else None,
            mesh=self.mesh.copy() if self.shape is None and self.mesh is not None else None,
            grid=self.attributes["grid"],
            box_mode=self.attributes.get("box_mode", True),
            name=self.attributes.get("name"),
            frame=self.frame.copy(),
        )
        part.key = self.key
        part.cell_network = self.cell_network.copy()
        return part
