# Copyright 2014--2017, John O. Woods, Ph.D.; West Virginia University Applied
# Space Exploration Laboratory; West Virginia Robotic Technology Center; and
# Intuitive Machines. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# Simple script demonstrating the computation of projected area on a
# fully convex, hollow, simple 3D object. See README.md for a partial
# description of why this isn't particularly useful for most use-cases.
#

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
