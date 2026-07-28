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

CellularizedPart does not assume the part is a brick / box:

- If the part's `shape` is an exact :class:`compas.geometry.Box` (e.g. a
  brick), the grid subdivides that box directly and every cell is `active`
  (the box is fully solid within its own bounding box).
- Otherwise (an arbitrary mesh, or a non-box shape), the grid subdivides the
  *bounding box* of the part's mesh, and each cell is tested against the
  actual mesh (by ray-casting containment of the cell's centroid) and marked
  `active=False` if it falls outside the mesh. The set of active cells then
  approximates the part's real, possibly irregular, volume rather than its
  plain bounding box -- a voxelization at the resolution of the chosen grid,
  not an exact boolean clip.

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
        self.ckey_to_ijk = {ckey: self.cell_attribute(ckey, "ijk") for ckey in self.cells()}
        self.ijk_to_ckey = {ijk: ckey for ckey, ijk in self.ckey_to_ijk.items()}

    def cell_at(self, i, j, k):
        """The cell key at grid index (i, j, k)."""
        return self.ijk_to_ckey[(i, j, k)]

    def active_cells(self):
        """Cell keys flagged `active` (part of the part's actual shape)."""
        return [ckey for ckey in self.cells() if self.cell_attribute(ckey, "active")]

    def build_cell_grid(self, box, grid=(2, 2, 2), mesh=None, tol=1e-6):
        """Fill this (empty) network with a regular grid of hexahedral
        cells subdividing `box`, in `box`'s own frame.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Reference box whose volume is subdivided into the grid. For a
            box-shaped part (e.g. a brick) this is the part's own exact
            shape. For an irregular part, pass its bounding box instead
            (e.g. ``part.mesh.compute_aabb()``).
        grid : tuple[int, int, int], optional
            Number of subdivisions along the box's local (x, y, z) axes,
            e.g. (2, 2, 2) -> 8 cells, (4, 2, 2) / (2, 2, 4) -> 16 cells.
        mesh : :class:`compas.datastructures.Mesh`, optional
            If given, each cell is tested against this mesh (by ray-casting
            containment of the cell's centroid) and flagged inactive if its
            centroid falls outside the mesh. Use this for irregular / non-box
            shapes so the active cells approximate the mesh's actual
            (possibly irregular) volume rather than its plain bounding box.
            This is a voxelization at the resolution of `grid`, not an exact
            boolean clip: cells straddling the mesh boundary are kept or
            dropped based on their centroid only.
        tol : float, optional
            Tolerance forwarded to the containment ray-cast.

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

            if mesh is not None:
                active = _point_in_mesh(self.cell_centroid(ckey), mesh, tol=tol)
            else:
                active = True
            self.cell_attribute(ckey, "active", active)

        self._reindex()
        return self


class CellularizedPart(Part):
    """A Part with an attached :class:`PartCellNetwork` cell grid.

    Does not assume the part is a brick: pass a `shape` (e.g. a
    :class:`compas.geometry.Box` for a brick) and/or a `mesh`. If `shape` is
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
    name : str, optional
    frame : :class:`compas.geometry.Frame`, optional
    """

    def __init__(self, shape=None, mesh=None, grid=(2, 2, 2), name=None, frame=None, **kwargs):
        super(CellularizedPart, self).__init__(name=name, frame=frame, **kwargs)

        if shape is not None:
            self.shape = shape
            self.mesh = mesh if mesh is not None else Mesh.from_shape(shape)
        elif mesh is not None:
            self.mesh = mesh
        else:
            raise ValueError("CellularizedPart needs a `shape` and/or a `mesh` to build its cell grid from.")

        self.attributes["grid"] = tuple(grid)
        self.cell_network = PartCellNetwork()
        self.build_cell_grid(grid=grid)

    @classmethod
    def from_box(cls, size, grid=(2, 2, 2), name=None, frame=None, **kwargs):
        """Convenience constructor for a box-shaped part, e.g. a brick.

        Parameters
        ----------
        size : tuple[float, float, float]
            Box (length, width, height).
        grid : tuple[int, int, int], optional
        name : str, optional
        frame : :class:`compas.geometry.Frame`, optional

        Returns
        -------
        :class:`CellularizedPart`
        """
        frame = frame or Frame.worldXY()
        box = Box(size[0], size[1], size[2], frame=frame.copy())
        return cls(shape=box, grid=grid, name=name, frame=frame, **kwargs)

    # ------------------------------------------------------------------
    # Cell grid
    # ------------------------------------------------------------------

    def build_cell_grid(self, grid=None):
        """(Re)build `self.cell_network` from the part's current shape/mesh.

        Parameters
        ----------
        grid : tuple[int, int, int], optional
            Defaults to the grid the part was last built with.
        """
        grid = tuple(grid) if grid is not None else self.attributes["grid"]
        self.attributes["grid"] = grid

        if isinstance(self.shape, Box):
            box = self.shape
            containment_mesh = None
        else:
            box = self.mesh.compute_aabb()
            containment_mesh = self.mesh

        self.cell_network.build_cell_grid(box, grid=grid, mesh=containment_mesh)

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
            name=self.attributes.get("name"),
            frame=self.frame.copy(),
        )
        part.key = self.key
        part.cell_network = self.cell_network.copy()
        return part
