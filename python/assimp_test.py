from pyassimp import *
import numpy as np
import matplotlib.pyplot as plt

def shoelace_area(vertices):
    pos = 0.0
    neg = 0.0
    for ii in range(0,vertices.shape[1]-1):
        pos += vertices[ii,0] * vertices[ii+1,1]
        neg += vertices[ii,1] * vertices[ii+1,0]
    return abs(pos - neg)

def projected_area_estimate(faces, T):

    vertices = np.dot(scene.meshes[0].vertices, np.transpose(T))
    normals  = np.dot(scene.meshes[0].normals,  np.transpose(T))

    area = 0.0

    fig = plt.figure(1)
    
    for ff in range(0,faces.shape[0]):
        vertex_ids    = faces[ff]

        face_vertices = vertices[vertex_ids,0:2] # put in 2D
        face_normals  = normals[vertex_ids]
    
        face_normal   = face_normals.mean(0)

        if face_normal[2] > 0.0:
            area += shoelace_area(face_vertices)

        print "Plotting face", ff
        plt.plot(face_vertices[:,0], face_vertices[:,1], lw=0.5)

    plt.show()
    return area

scene = load('AXHM_V2.obj')

if len(scene.meshes) != 1:
    print "Error: This program only works for a single mesh."
    exit

faces = scene.meshes[0].faces

print "Area =", projected_area_estimate(faces, np.identity(3))
