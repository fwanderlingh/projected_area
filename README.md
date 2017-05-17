# projected_area #

This program uses OpenGL to quickly compute the projected area of an
object, which is useful for computing solar radiation pressure and
drag.

* [John O. Woods, Ph.D](http://github.com/mohawkjohn) &mdash; john.o.woods@gmail.com
* [Intuitive Machines](http://www.intuitivemachines.com)

## Description ##

As of this writing, no publicly available program exists to compute
the projected area of a 3-dimensional object from any viewing
angle. While a tutorial exists for computing this value through
SolidWorks, that functionality cannot be easily automated. I wanted to
be able to compute the drag cross-section from any arbitrary angle in
order to do orbital simulations, so I wrote this simple application.

There are probably more efficient ways to achieve this goal, but I
needed something quick, dirty, and robust. (I wrote this application
in about four hours, basing it off of
[GLIDAR](http://github.com/wvu-asel/glidar), which I wrote several
years ago for simulating 3D cameras and LIDARs.)

### How else might I do this? ###

Here are the steps for a more computationally efficient solution than is given here:

1. Load the object using ASSIMP and also compute normals if none are given.
2. Rotate the object's vertices and normals to the desired attitude.
3. Delete any faces whose mean normal is away from the viewer, as these will be invisible.
4. Either:
  1. Perform [hidden surface removal of some sort](http://en.wikipedia.org/wiki/Hidden_surface_determination) to hide any occluded faces whose normals are toward the viewer, so these aren't counted twice, then return the remaining polygons without their z-coordinate; or,
  2. Paying attention only to x and y coordinates and neglecting z, merge all of the vertices into a single 2D polygon.
5. Use Gauss' [Shoelace algorithm](https://en.wikipedia.org/wiki/Shoelace_formula) to compute the are of the surviving 2D polygon or polygons.

Feel free to submit a pull request for this solution! Or if you solve it in a separate repository, I'd be grateful for a link, which I should happily include here.

### Why use OpenGL, then? ###

The main reason is that I already wrote a simple version of the above,
which neglects step 4, and it was too slow (granted, I wrote it in
Python, not C++). You can find this code in the python subdirectory of
this repository. It will work just fine if your 3D object happens to
be (a) hollow, and (b) convex or very nearly convex.

OpenGL is nice because on modern computers it can take advantage of
the GPU. So while the runtime complexity may be unpleasant, a solution
can be obtained quickly.

## Requirements ##

Since I based this code off GLIDAR, it should work fine on OS X
(Mavericks and Yosemite, at least). However, I developed *this*
application on Ubuntu, and haven't tested extensively; so you may need
to tweak it a bit. With that said, it's really just a subset of
GLIDAR's code, so you should be fine.

GLIDAR has been tested on Mac OS X Mavericks and Yosemite
installations with Homebrew, and on a couple of Ubuntu (Trusty and
later) machine. It has the following requirements:

* CMake (2.6 or higher is expected in CMakeLists.txt)
* GLSL 1.2* support (in graphics card)
* GLFW 3
* GLEW 1.10 (tested with 1.13 in this case)
* GLM 0.9.6+ (tested with 0.9.7.2-1 in this case)
* [ASSIMP](http://assimp.sourceforge.net/) (tested with 3.2), which may actually be unnecessary.

The Python code in that subdirectory needs:

* Python 2.7
* numpy 1.10 or so
* ASSIMP's Python API (see the ASSIMP repository for details)
* matplotlib
* cPickle (though you can comment this part out or have it use regular Pickle instead)
* [pyquat](http://github.com/mohawkjohn/pyquat)

### Dependency Installation ###

On Ubuntu, you probably want to do something like

    sudo apt-get install libassimp-dev libglfw3-dev libglew-dev libglm-dev cmake

For those who are Mac OS X users, I recommend using homebrew to
install any dependencies.

## Installation ##

First, you need a local copy of the repository:

    git clone https://github.com/mohawjohn/projected_area.git
    cd projected_area
    
An in-tree build is not advised. As such, you should create a `build`
subdirectory and `cd` into it, then tell `cmake` where to find the
source tree.
    
    mkdir build
    cd build
    cmake ..
    
If cmake reported no error messages, you can now build:

    make
    
You can probably also safely do a

    sudo make install

but I haven't actually tested that.

## Usage ##

### Example Usage: Linux/Unix ###

Suppose you want to open <tt>myobject.obj</tt>, where the origin is
more or less centered, and the largest dimension is under 10.0
meters. Suppose you also want the window size to be 256 by 256 pixels.

    build/projected_area myobject.obj 10.0 256 -p

You'll then want to enter the quaternion for the attitude you want to
print, separated by spaces, and concluding with the enter key. It will
render that attitude. You can do this over and over again. Type 'x'
followed by ENTER when you're done.

The <tt>-p</tt> flag tells the OpenGL display to persist even after
all of the attitudes have been rendered. If you use this flag, exit
using Ctrl+C.

Note also that the program will normalize any quaternion you give it,
which is handy if you don't feel like typing out all the digits of an
irrational number. For example, you can just use <tt>1 0 1 0</tt>,
which it interprets as a ratio.

There is also a Python script which demonstrates the computation of as
many projected areas as you'd like:

    python python/sphere.py

The parameters are stored in that script.


### Running on Mac OS X ###

Run it exactly the same way as in Linux, but instead of using
`build/projected_area` as the binary, use
`build/projected_area.app/Contents/MacOS/projected_area`. Having it packaged in this
way helps the program to interact with Finder.

### Known Issues ###

The greatest limitation right now is in the ASSIMP library, which
loads the 3D models. I have not been able to get it to work with the
[full high-resolution International Space Station model](http://nasa3d.arc.nasa.gov/detail/iss-hi-res), for example
&mdash; but it should load individual modules of the ISS properly.

## Bug Reports ##

If you find a bug, please file it in our [issues tracker](https://github.com/mohawkjohn/projected_area/issues).
  
## Contributing ##

Contributions are appreciated. We prefer that you submit them as pull
requests to the [projected_area repository](https://github.com/mohawkjohn/projected_area). If
you don't know how to use `git rebase`, please feel free to ask.

## License Note ##

Note that the license includes WVU and WVRTC copyright because it is
derived from GLIDAR. This derivative work is owned by John O. Woods,
Ph.D., and Intuitive Machines.

## License ##

Copyright 2014--2017, John O. Woods, Ph.D.; West Virginia University Applied
Space Exploration Laboratory; West Virginia Robotic Technology Center; and
Intuitive Machines. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
