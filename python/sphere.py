# Copyright 2017, John O. Woods, Ph.D., and Intuitive Machines. All
# rights reserved.
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
# Call projected_area on all the points on a sphere to generate a
# surface area contour map.
#

import numpy as np
import matplotlib.pyplot as plt
import pyquat
from pyquat import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as colors
import matplotlib.cm as cm
import subprocess
import cPickle as pickle

args = ["build/projected_area", "AXHM_V2.obj", "10"]

u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
x = np.cos(u) * np.sin(v)
y = np.sin(u) * np.sin(v)
z = np.cos(v)

q0 = pyquat.identity()

output = np.zeros_like(x)

for ii in range(0, x.shape[0]):
    for jj in range(0, x.shape[1]):
        vz = np.transpose(-np.array([x[ii,jj], y[ii,jj], z[ii,jj]]))
        q = Quat(0.0, *vz)
        this_args = args[:] + ["0.0", str(vz[0]), str(vz[1]), str(vz[2])]
        this_args
        print "This_args =", this_args
        popen = subprocess.Popen(this_args, stdout=subprocess.PIPE)
        popen.wait()
        output[ii,jj] = float(popen.stdout.readline().rstrip())
        x[ii,jj] *= output[ii,jj]
        y[ii,jj] *= output[ii,jj]
        z[ii,jj] *= output[ii,jj]


pickle.dump( {"x": x, "y": y, "z": z, "area": output}, open("area.p", "wb"))
fig = plt.figure()
axis = fig.add_subplot(111, projection = '3d')
axis.set_title("Cross-sectional area according to velocity vector")
axis.set_xlabel("x")
axis.set_ylabel("y")
axis.set_zlabel("z")

color_map = cm.jet((output - output.min()) / (output.max() - output.min()))
surface = axis.plot_surface(x, y, z, facecolors=color_map, rstride=1, cstride=1)
plt.show()
