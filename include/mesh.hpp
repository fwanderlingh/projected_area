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
 ** @brief Mesh for a 3D object we wish to render, which we have loaded via Assimp.
 **        This source code is based on GLIDAR, and that in turn loosely on an
 **        example given by Etay Meiri.
 **
 **/

#ifndef MESH_HPP
# define MESH_HPP

#include <vector>
#include <iostream>

#include "shader.hpp"
//#include "vertex.hpp"

#include <glm/gtc/type_ptr.hpp> // glm::value_ptr
#include <glm/gtx/projection.hpp> // glm::proj
#include <glm/gtx/string_cast.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <flann/flann.hpp>
#include <flann/algorithms/kdtree_single_index.h>

#define INVALID_OGL_VALUE 0xFFFFFFFF
const size_t MAX_LEAF_SIZE = 16;
const float MIN_NEAR_PLANE = 0.01; // typically meters, but whatever kind of
                                   // distance units you're using for your world

/** @brief Mesh class for loading 3D objects via Assimp.
 **/
class Mesh 
{
public:
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

  

  Mesh() : min_extremities(0.0f,0.0f,0.0f), max_extremities(0.0f,0.0f,0.0f) { }

  glm::vec3 dimensions() const {
    return max_extremities - min_extremities;
  }


  bool load_mesh(const std::string& filename);

  void render(Shader* shader_program);
  
  
  
  /** Get the centroid of the object, an average of the positions of all vertices.
   *
   */
  glm::vec3 centroid() const {
    return centroid_;
  }


  float nearest_point(const glm::vec4& p, glm::vec4& result) const;
  float near_plane_bound(const glm::mat4& model_to_object_coords, const glm::vec4& camera_pos) const;
  float far_plane_bound(const glm::mat4& model_to_object_coords, const glm::vec4& camera_pos) const;


protected:
  void init_mesh(const aiScene* scene, const aiMesh* mesh, size_t index);

  bool init_from_scene(const aiScene* scene, const std::string& filename);

  class MeshEntry 
  {
  public:
    MeshEntry()
      : vb(INVALID_OGL_VALUE), 
        ib(INVALID_OGL_VALUE), 
        num_indices(0),
        xyz_data(NULL),
        xyz(NULL),
        kdtree(NULL),
        flann_epsilon(0.0),
        flann_sorted(true),
        flann_checks(32)
    { }

    ~MeshEntry() {
      if (vb != INVALID_OGL_VALUE) glDeleteBuffers(1, &vb);
      if (ib != INVALID_OGL_VALUE) glDeleteBuffers(1, &ib);

      // Delete the space allocated within xyz, then delete xyz container, then delete the kdtree.
      delete [] xyz_data;
      delete xyz;
      delete kdtree;
    }

    void init(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
      num_indices = indices.size();

      glGenBuffers(1, &vb);
      glBindBuffer(GL_ARRAY_BUFFER, vb);
      glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), &vertices[0], GL_STATIC_DRAW);

      glGenBuffers(1, &ib);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, &indices[0], GL_STATIC_DRAW);

      // Copy the xyz coordinates from vertices into the xyz array.
      xyz_data = new float[vertices.size() * 3];
      for (size_t i = 0; i < vertices.size(); ++i) {
	xyz_data[i*3+0] = vertices[i].x();
	xyz_data[i*3+1] = vertices[i].y();
        xyz_data[i*3+2] = vertices[i].z();
      }
      // Create the matrix
      xyz = new flann::Matrix<float>(xyz_data, vertices.size(), 3);

      // Create a k-D tree, which we will use to find the near plane no matter how the object is rotated.
      kdtree = new flann::KDTreeSingleIndex<flann::L2_Simple<float> >(*xyz, flann::KDTreeSingleIndexParams(MAX_LEAF_SIZE));
      kdtree->buildIndex();
    }

    /** Returns the nearest point in model coordinates to a given point.
     *
     * @param[in] The query point.
     * @param[out] The nearest point found to the query point.
     *
     * \returns The distance from the query point to the nearest mesh point.
     */
    float nearest_point(const glm::vec4& p, glm::vec4& result) const {
      flann::Matrix<float> query(const_cast<float*>(static_cast<const float*>(glm::value_ptr(p))), 1, 3);

      // Result matrices
      int index;
      float distance;
      flann::Matrix<int> indices(&index, 1, 1);
      flann::Matrix<float> distances(&distance, 1, 1);

      flann::SearchParams search_parameters;
      search_parameters.eps = flann_epsilon;
      search_parameters.sorted = flann_sorted;
      search_parameters.checks = flann_checks;


      kdtree->knnSearch(query, indices, distances, 1, search_parameters);

      result.x = xyz_data[index*3+0];
      result.y = xyz_data[index*3+1];
      result.z = xyz_data[index*3+2];
      result.w = 0.0;

      return distance;
    }


    GLuint vb;
    GLuint ib;
    size_t num_indices;

    float* xyz_data;
    flann::Matrix<float>* xyz;
    flann::KDTreeSingleIndex<flann::L2_Simple<float> >* kdtree;
    float flann_epsilon;
    bool  flann_sorted;
    int   flann_checks;
  };

  std::vector<MeshEntry> entries;
  glm::vec3 min_extremities, max_extremities, centroid_;
  
};




#endif // MESH_HPP
