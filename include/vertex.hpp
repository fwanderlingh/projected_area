/*
 * Copyright (c) 2014 - 2017, John O. Woods, Ph.D.
 *   West Virginia University Applied Space Exploration Lab (2014 - 2015)
 *   West Virginia Robotic Technology Center (2014 - 2015)
 *   Intuitive Machines (2017)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 ** @file
 **
 ** @brief Vertex class for Mesh. This source code is based on GLIDAR, and that
 **        in turn loosely on an example given by Etay Meiri.
 **
 **/

#ifndef MESH_HPP
# define MESH_HPP

#include <glm/glm.hpp>

class Vertex
{
public:
  glm::vec3 pos;
  //glm::vec3 normal;

  Vertex() {}

  Vertex(const glm::vec3& pos_) //, const glm::vec3& normal_)
  : pos(pos_) //, diffuse_tex(dtex_), specular_tex(stex_), normal(normal_)
  {  }

  float x() const { return pos.x; }
  float y() const { return pos.y; }
  float z() const { return pos.z; }
};



#endif // MESH_HPP
