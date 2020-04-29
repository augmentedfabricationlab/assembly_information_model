from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from math import fabs

import compas

try:
    from numpy import array
    from numpy import float64

    from scipy.linalg import solve
    from scipy.spatial import cKDTree
except ImportError:
    compas.raise_if_not_ironpython()

try:
    from shapely.geometry import Polygon
except ImportError:
    compas.raise_if_not_ironpython()

from compas.geometry import Frame
from compas.geometry import local_to_world_coords_numpy
# from compas.geometry import project_points_plane
# from compas.geometry import centroid_points


__all__ = [
    'assembly_interfaces_numpy',
]


def _find_nearest_neighbours(cloud, nmax):
    tree = cKDTree(cloud)
    nnbrs = [tree.query(root, nmax) for root in cloud]
    nnbrs = [(d.flatten().tolist(), n.flatten().tolist()) for d, n in nnbrs]
    return nnbrs

def assembly_interfaces_numpy_fromtojson(PATH_FROM,
                              PATH_TO, 
                              nmax=10,
                              tmax=1e-6,
                              amin=1e-1,
                              lmin=1e-3,
                              face_face=True,
                              face_edge=False,
                              face_vertex=False):
    
    
    # load an assembly from JSON
    from .assembly import Assembly
    assembly = Assembly.from_json(PATH_FROM)
    assembly_interfaces_numpy(assembly,nmax=nmax,tmax=tmax,amin=amin,lmin=lmin,face_face=face_face,face_edge=face_edge,face_vertex=face_vertex)
    assembly.to_json(PATH_TO)

def assembly_interfaces_numpy(assembly,
                              nmax=10,
                              tmax=1e-6,
                              amin=1e-1,
                              lmin=1e-3,
                              face_face=True,
                              face_edge=False,
                              face_vertex=False):
    """Identify the interfaces between the blocks of an assembly.

    Parameters
    ----------
    assembly : compas_assembly.datastructures.Assembly
        An assembly of discrete blocks.
    nmax : int, optional
        Maximum number of neighbours per block.
        Default is ``10``.
    tmax : float, optional
        Maximum deviation from the perfectly flat interface plane.
        Default is ``1e-6``.
    amin : float, optional
        Minimum area of a "face-face" interface.
        Default is ``1e-1``.
    lmin : float, optional
        Minimum length of a "face-edge" interface.
        Default is ``1e-3``.
    face_face : bool, optional
        Test for "face-face" interfaces.
        Default is ``True``.
    face_edge : bool, optional
        Test for "face-edge" interfaces.
        Default is ``False``.
    face_vertex : bool, optional
        Test for "face-vertex" interfaces.
        Default is ``False``.

    References
    ----------
    The identification of interfaces is discussed in detail here [Frick2016]_.

    Examples
    --------
    .. code-block:: python

        pass

    """
    # replace by something proper
    assembly.network.edge = {}
    assembly.network.halfedge = {}
    for key in assembly.network.nodes():
        assembly.network.edge[key] = {}
        assembly.network.halfedge[key] = {}

    key_index = assembly.network.key_index()
    index_key = assembly.network.index_key()

    blocks = [assembly.element(key) for key in assembly.network.nodes()]
    nmax = min(nmax, len(blocks))
    block_cloud = assembly.network.nodes_attributes('xyz')
    block_nnbrs = _find_nearest_neighbours(block_cloud, nmax)

    # k:      key of the base block
    # i:      index of the base block
    # block:  base block
    # nbrs:   list of indices of the neighbouring blocks
    # frames: list of frames for each of the faces of the base block

    # f0:   key of the current base face
    # A:    uvw base frame of f0
    # o:    origin of the base frame of f0
    # xyz0: xyz coordinates of the nodes of f0
    # rst0: local coordinates of the nodes of f0, with respect to the frame of f0
    # p0:   2D polygon of f0 in local coordinates

    # j:   index of the current neighbour
    # n:   key of the current neighbour
    # nbr: neighbour block
    # k_i: key index map for the nodes of the nbr block
    # xyz: xyz coorindates of all nodes of nbr
    # rst: local coordinates of all nodes of nbr, with respect to the frame of f0

    # f1:   key of the current neighbour face
    # rst1: local coordinates of the nodes of f1, with respect to the frame of f0
    # p1:   2D polygon of f1 in local coordinates

    for k in assembly.network.nodes():

        i = key_index[k]

        block = assembly.element(k)
        nbrs = block_nnbrs[i][1]

        frames = block.face_frames

        if face_face:

            # parallelise?
            # exclude faces with parallel normals
            # e.g. exclude overlapping top faces of two neighbouring blocks in same row

            for f0, (origin, uvw) in frames.items():
                A = array(uvw, dtype=float64)
                o = array(origin, dtype=float64).reshape((-1, 1))
                xyz0 = array(block._mesh.face_coordinates(f0), dtype=float64).reshape((-1, 3)).T
                rst0 = solve(A.T, xyz0 - o).T.tolist()
                p0 = Polygon(rst0)

                for j in nbrs:
                    n = index_key[j]

                    if n == k:
                        continue

                    if k in assembly.network.edge and n in assembly.network.edge[k]:
                        continue

                    if n in assembly.network.edge and k in assembly.network.edge[n]:
                        continue

                    nbr = assembly.element(n)
                    #print(nbr)
                    k_i = {key: index for index, key in enumerate(nbr._mesh.vertices())}
                    xyz = array(nbr._mesh.vertices_attributes('xyz'), dtype=float64).reshape((-1, 3)).T
                    rst = solve(A.T, xyz - o).T.tolist()
                    rst = {key: rst[k_i[key]] for key in nbr._mesh.vertices()}

                    for f1 in nbr._mesh.faces():

                        rst1 = [rst[key] for key in nbr._mesh.face_vertices(f1)]

                        if any(fabs(t) > tmax for r, s, t in rst1):
                            continue

                        p1 = Polygon(rst1)

                        if p1.area < amin:
                            continue

                        if p0.intersects(p1):
                            intersection = p0.intersection(p1)

                            area = intersection.area

                            if area >= amin:
                                coords = [[x, y, 0.0] for x, y, z in intersection.exterior.coords]
                                coords = local_to_world_coords_numpy(Frame(o, A[0], A[1]), coords)

                                attr = {
                                    'interface_type': 'face_face',
                                    'interface_size': area,
                                    'interface_points': coords.tolist()[:-1],
                                    'interface_origin': origin,
                                    'interface_uvw': uvw,
                                }

                                assembly.network.add_edge(k, n, attr_dict=attr)


if __name__ == "__main__":
    pass
