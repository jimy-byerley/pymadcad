import numpy as np


from madcad.prelude import *
import madcad as cad


def draft_extrusion(mesh: Mesh, trans: vec3, angle: float) -> Mesh:
    ex = cad.extrusion(trans, mesh)
    ex.finish()
    trans_faces = ex.group(2).faces
    face_arr = np.array([face.to_tuple() for face in trans_faces])
    trans_inds = np.unique(face_arr)
    
    tube = ex.group(0)
    n_trans = normalize(trans)
    trans_len = np.linalg.norm(trans)   

    face_inds = [i for i, face in enumerate(tube.faces )if 2 >len(set(face) - set(trans_inds))]
    tube_normals = tube.facenormals()
    derp = []
    for i in trans_inds:
        faces = [tube.faces[face_i] for face_i in face_inds if i in tube.faces[face_i]]
        normals = [tube_normals[face_i] for face_i in face_inds if i in tube.faces[face_i]]
        c1 = cross(*normals)
        c2 = cross(c1, normals[0])
        c3 = cross(normals[1], c1)
        mixed = (c2 + c3 + sum(normals)) / 2
        projected = mixed - n_trans*dot(mixed, n_trans)
        offset = projected*trans_len * np.sin(np.deg2rad(angle))
        ex.points[i] += offset


    return ex
    
    

if __name__== "__main__":
    m = cad.square(Axis((0,0,0), Z), 2)
    print(m.points)
    ex = draft_extrusion(m, -8*Z, 30)
    for p in ex.group(2).points:
        print(p)
    show([ex])
