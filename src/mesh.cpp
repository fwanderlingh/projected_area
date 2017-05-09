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

/** @file
 **
 ** @brief Implementation for Assimp-loaded Mesh. See notes on mesh.hpp.
 **/

#include "../include/mesh.hpp"

void Mesh::render(Shader* shader_program) {

  shader_program->bind();

  check_gl_error();

  GLuint position_loc     = glGetAttribLocation(shader_program->id(), "position");
  //GLuint normal_loc       = glGetAttribLocation(shader_program->id(), "normal");
  check_gl_error();

  glEnableVertexAttribArray(position_loc);
  check_gl_error();
  //glEnableVertexAttribArray(normal_loc);
  //check_gl_error();

  for (size_t i = 0; i < entries.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, entries[i].vb);

    // I think this tells it where to look for the vertex information we've loaded.
    glVertexAttribPointer(position_loc,           3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
    //glVertexAttribPointer(normal_loc,             3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)28); // makes room for 7 floats

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, entries[i].ib);

    //glColor4f(1.0, 1.0, 1.0, 1.0);
    glDrawElements(GL_TRIANGLES, entries[i].num_indices, GL_UNSIGNED_INT, 0);
  }

  check_gl_error();

  glDisableVertexAttribArray(position_loc);
  //glDisableVertexAttribArray(normal_loc);

  check_gl_error();

  shader_program->unbind();
}

void Mesh::init_mesh(const aiScene* scene, const aiMesh* mesh, size_t index) {

  std::cerr << "Loading mesh named '" << mesh->mName.C_Str() << "'" << std::endl;

  if (!mesh->HasNormals()) {
    std::cerr << "Mesh has no normals!" << std::endl;
  } else {
    if (mesh->mNumVertices > 0)
      std::cerr << "First normal:" << mesh->mNormals[0].x << "," << mesh->mNormals[0].y << "," << mesh->mNormals[0].z << std::endl;
  }

  // Create vectors to store the transformed position and normals; initialize them to the origin.
  std::vector<aiVector3D> final_pos(mesh->mNumVertices);
//                          final_normal(mesh->mNumVertices);
  for (size_t i = 0; i < mesh->mNumVertices; ++i) {
    final_pos[i] = aiVector3D(0.0, 0.0, 0.0); // final_normal[i] = 
  }


  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  const aiVector3D zero_3d(0.0, 0.0, 0.0);


  if (mesh->mNumBones) {
    std::vector<aiMatrix4x4> bone_matrices(mesh->mNumBones);

    // Calculate bone matrices.
    for (size_t i = 0; i < mesh->mNumBones; ++i) {
      const aiBone* bone = mesh->mBones[i];
      bone_matrices[i] = bone->mOffsetMatrix;

      std::cerr << "Bone '" << bone->mName.C_Str() << "' includes " << bone->mNumWeights << " vertices:" << std::endl;
      for (size_t j = 0; j < bone->mNumWeights; ++j) {
        std::cerr << ' ' << bone->mWeights[j].mVertexId;
      }
      std::cerr << std::endl;

      const aiNode* node = scene->mRootNode->FindNode(bone->mName.C_Str());
      const aiNode* temp_node = node;
      while (temp_node != NULL) {
        bone_matrices[i] = temp_node->mTransformation * bone_matrices[i];
        temp_node = temp_node->mParent;
      }
    }

    // Update vertex positions according to calculated matrices
    for (size_t i = 0; i < mesh->mNumBones; ++i) {
      const aiBone* bone = mesh->mBones[i];
      //aiMatrix3x3 normal_matrix = aiMatrix3x3(bone_matrices[i]);

      for (size_t j = 0; j < bone->mNumWeights; ++j) {
        const aiVertexWeight *vertex_weight = &(bone->mWeights[j]);
        size_t v = (size_t)(vertex_weight->mVertexId);
        float  w = vertex_weight->mWeight;

        const aiVector3D *src_pos = &(mesh->mVertices[v]);
        //const aiVector3D *src_normal = &(mesh->mNormals[v]);

        final_pos[v]    += w * (bone_matrices[i] * (*src_pos));
        //final_normal[v] += w * (normal_matrix * (*src_normal));
      }

      std::cerr << "bone " << i << ":" << std::endl;
      std::cerr << bone_matrices[i].a1 << ' ' << bone_matrices[i].a2 << ' ' << bone_matrices[i].a3 << ' ' << bone_matrices[i].a4 << std::endl;
      std::cerr << bone_matrices[i].b1 << ' ' << bone_matrices[i].b2 << ' ' << bone_matrices[i].b3 << ' ' << bone_matrices[i].b4 << std::endl;
      std::cerr << bone_matrices[i].c1 << ' ' << bone_matrices[i].c2 << ' ' << bone_matrices[i].c3 << ' ' << bone_matrices[i].c4 << std::endl;
      std::cerr << bone_matrices[i].d1 << ' ' << bone_matrices[i].d2 << ' ' << bone_matrices[i].d3 << ' ' << bone_matrices[i].d4 << std::endl;
    }

    // initialize our dimension trackers.
    if (mesh->mNumVertices != 0) {
      min_extremities.x = final_pos[0].x;
      min_extremities.y = final_pos[0].y;
      min_extremities.z = final_pos[0].z;
      max_extremities = min_extremities;
    }

    // Add each updated vertex and calculate its extremities.
    for (size_t i = 0; i < mesh->mNumVertices; ++i) {
      // Find the extremities of this mesh so we can get a measurement for the object in object units.
      if (final_pos[i].x < min_extremities.x) min_extremities.x = final_pos[i].x;
      else if (final_pos[i].x > max_extremities.x) max_extremities.x = final_pos[i].x;

      if (final_pos[i].y < min_extremities.y) min_extremities.y = final_pos[i].y;
      else if (final_pos[i].y > max_extremities.y) max_extremities.y = final_pos[i].y;

      if (final_pos[i].z < min_extremities.z) min_extremities.z = final_pos[i].z;
      else if (final_pos[i].z > max_extremities.z) max_extremities.z = final_pos[i].z;


      Vertex vertex(glm::vec3(final_pos[i].x, final_pos[i].y, final_pos[i].z));
      //         glm::vec3(final_normal[i].x, final_normal[i].y, final_normal[i].z));

      std::cerr << "Adding vertex " << i << ": " << final_pos[i].x << "," << final_pos[i].y << "," << final_pos[i].z;
//      std::cerr << "\t" << final_normal[i].x << "," << final_normal[i].y << "," << final_normal[i].z << std::endl;
      std::cerr << "  was: " << mesh->mVertices[i].x << "," << mesh->mVertices[i].y << "," << mesh->mVertices[i].z << std::endl;
//      std::cerr << mesh->mNormals[i].x << "," << mesh->mNormals[i].y << "," << mesh->mNormals[i].z << std::endl;

      // Accumulate the centroid_ of the object.
      centroid_ += vertex.pos;

      vertices.push_back(vertex);
    }
  } else {
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
      const aiVector3D* pos    = &(mesh->mVertices[i]);
      //const aiVector3D* normal = &(mesh->mNormals[i]);
      
      // Find the extremities of this mesh so we can get a measurement for the object in object units.
      if (pos->x < min_extremities.x)       min_extremities.x = pos->x;
      else if (pos->x > max_extremities.x)  max_extremities.x = pos->x;

      if (pos->y < min_extremities.y)       min_extremities.y = pos->y;
      else if (pos->y > max_extremities.y)  max_extremities.y = pos->y;

      if (pos->z < min_extremities.z)       min_extremities.z = pos->z;
      else if (pos->z > max_extremities.z)  max_extremities.z = pos->z;

      Vertex v(glm::vec3(pos->x, pos->y, pos->z));
      //    glm::vec3(normal->x, normal->y, normal->z));

      // Accumulate the centroid_ of the object.
      centroid_ += v.pos;

      vertices.push_back(v);
    }
  }

  centroid_ /= mesh->mNumVertices;

  // Add vertices for each face
  for (size_t i = 0; i < mesh->mNumFaces; ++i) {
    const aiFace& face = mesh->mFaces[i];
    if (face.mNumIndices != 3) {
      std::cerr << "Face has " << face.mNumIndices << " indices; skipping" << std::endl;
      continue;
    }
    indices.push_back(face.mIndices[0]);
    indices.push_back(face.mIndices[1]);
    indices.push_back(face.mIndices[2]);
  }


  // Create index buffer.
  entries[index].init(vertices, indices);
}


bool Mesh::load_mesh(const std::string& filename) 
{
  bool ret = false;
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(filename.c_str(), aiProcess_Triangulate | aiProcess_GenNormals );

  if (scene)    ret = init_from_scene(scene, filename);
  else std::cerr << "Error parsing '" << filename << "': " << importer.GetErrorString() << std::endl;

  return ret;
}


/** @brief Find the nearest point among all the meshes to some point p.
 *
 * @param[in] query point.
 * @param[out] result point.
 *
 * @returns a distance.
 */
float Mesh::nearest_point(const glm::vec4& p, glm::vec4& result) const {
  float d = entries[0].nearest_point(p, result);

  for (size_t i = 1; i < entries.size(); ++i) {
    glm::vec4 tmp_result;
    float tmp_distance;

    tmp_distance = entries[i].nearest_point(p, tmp_result);

    if (tmp_distance < d) {
      d = tmp_distance;
      result = tmp_result;
    }
  }

  return d;
}


/** @brief Find the ideal location of the near plane.
 *
 * @param[in] model coordinate matrix.
 * @param[in] camera position.
 *
 * @returns The distance from the camera to the ideal near plane if we didn't mind the plane intersecting the nearest point.
 */
float Mesh::near_plane_bound(const glm::mat4& model_to_object_coords, const glm::vec4& camera_pos) const {
  glm::vec4 nearest;

  // Find the nearest point in the mesh.
  nearest_point(camera_pos, nearest);
  nearest.w = 0.0;

  float bound = (model_to_object_coords * (camera_pos - nearest)).z;

  if (bound <= 0) {
    std::cerr << "WARNING: Nearest point on object is behind the sensor, which makes for an invalid near plane setting. Using MIN_NEAR_PLANE="
              << MIN_NEAR_PLANE << " distance units for the bound. Actual near plane will be slightly closer, depending on your value for NEAR_PLANE_FACTOR." 
              << std::endl;
    bound = MIN_NEAR_PLANE;
  }

  return bound;
}


/** @brief Find the ideal location of the far plane.
 *
 * @param[in] model coordinate matrix.
 * @param[in] camera position.
 *
 * @returns The distance from the camera to the ideal far plane if we didn't mind the plane intersecting the farthest point.
 */
float Mesh::far_plane_bound(const glm::mat4& model_to_object_coords, const glm::vec4& camera_pos) const {
  // Find the farthest point in the mesh
  glm::vec4 negative_camera_pos(-camera_pos);
  glm::vec4 farthest;
  nearest_point(negative_camera_pos, farthest);
  farthest.w = 0.0;
  return (model_to_object_coords * (camera_pos - farthest)).z;
}



bool Mesh::init_from_scene(const aiScene* scene, const std::string& filename) {
  entries.resize(scene->mNumMeshes);

  std::cerr << "Reading " << entries.size() << " meshes" << std::endl;

  // Initialize the meshes in the scene one by one
  for (size_t i = 0; i < entries.size(); ++i) {
    const aiMesh* mesh = scene->mMeshes[i];
    init_mesh(scene, mesh, i);
  }

  return true;
}
