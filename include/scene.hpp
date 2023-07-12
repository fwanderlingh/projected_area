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
 ** @brief Headers for a scene loaded via Assimp. Based on GLIDAR.
 **
 **/

#ifndef SCENE_HPP
# define SCENE_HPP

#define GLM_FORCE_RADIANS

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/string_cast.hpp>
#include <cmath>
#include "mesh.hpp"

#define _USE_MATH_DEFINES

const unsigned int BOX_HALF_DIAGONAL = 174;
const double RADIANS_PER_DEGREE = M_PI / 180.0;
const float NEAR_PLANE_FACTOR = 0.99;
const float FAR_PLANE_FACTOR = 1.01;



/** @brief Simple object and sensor OpenGL scene, which handles loading and
 **        rendering meshes, and also computing the projected area.
 */
class Scene {
public:
  
  /** Constructor for the scene, which does most of the rendering work.
   *
   * This function sets near_plane_bound, real_near_plane, and far_plane, which end up not being used after initial setup.
   * Eventually they should be removed.
   *
   * @param[in] 3D model file to load.
   * @param[in] amount by which to scale the model we load.
   * @param[in] initial camera distance.
   */
  Scene(const std::string& filename, float scale_factor_, float camera_d_, float box_width_in)
  : scale_factor(scale_factor_),
    projection(1.0),
    camera_d(camera_d_),
    near_plane_bound(camera_d_ - BOX_HALF_DIAGONAL),
    real_near_plane(std::max(MIN_NEAR_PLANE, camera_d_-BOX_HALF_DIAGONAL)),
    far_plane(camera_d_+BOX_HALF_DIAGONAL),
    box_width(box_width_in)
  {
    std::cerr << "camera_d = " << camera_d << std::endl;
    mesh.load_mesh(filename);

    glm::vec3 dimensions = mesh.dimensions();
    std::cerr << "Object dimensions as modeled: " << dimensions.x << '\t' << dimensions.y << '\t' << dimensions.z << std::endl;
    glm::vec3 centroid = mesh.centroid();
    std::cerr << "Center of object as modeled: " << centroid.x << '\t' << centroid.y << '\t' << centroid.z << std::endl;
  }


  /** Set OpenGL options.
   *
   */
  void gl_setup() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_NORMALIZE);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glDepthFunc(GL_LEQUAL);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);

    glPolygonMode( GL_FRONT, GL_FILL );
  }



  /** Setup the perspective projection matrix, and figure out where to draw the near and far planes.
   *
   * @param[in] Field of view of the sensor.
   * @param[in] Model matrix inverse.
   * @param[in] View matrix.
   */
  void projection_setup(const glm::mat4& inverse_model, const glm::mat4& view_physics) {
    gl_setup();

    glm::mat4 inverse_view = glm::inverse(view_physics);
    glm::vec4 camera_pos_mc = inverse_model * inverse_view * glm::vec4(0.0, 0.0, 0.0, 1.0);
    glm::mat4 model = glm::inverse(inverse_model);
    
    near_plane_bound = mesh.near_plane_bound(model, camera_pos_mc);
    real_near_plane = near_plane_bound * NEAR_PLANE_FACTOR;
    far_plane = mesh.far_plane_bound(model, camera_pos_mc) * FAR_PLANE_FACTOR;

    GLfloat w = box_width / 2.0;
    projection = glm::ortho(-w, w, -w, w, near_plane_bound, far_plane);
  }


  /** Render the scene using the model and view matrices.
   *
   *Sets up the model view matrix, calls projection_setup, calculates a normal matrix.
   *
   * @param[in] the GLSL shader program.
   * @param[in] the field of view of the sensor.
   * @param[in] model matrix inverse.
   * @param[in] view matrix.
   */
  void render(Shader* shader_program, const glm::mat4& inverse_model_in, const glm::mat4& view_in) {
    glm::mat4 inverse_model =
      glm::scale(
        glm::mat4(1),
        glm::vec3(1.0/scale_factor, 1.0/scale_factor, 1.0/scale_factor)
        ) * inverse_model_in;
    
    projection_setup(inverse_model, view_in);

    // clear window with the current clearing color, and clear the depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shader_program->id());

    glm::mat4 model_view_projection = projection * view_in * glm::inverse(inverse_model);
    
    GLint mvp_id = glGetUniformLocation(shader_program->id(), "ModelViewProjectionMatrix");
    glUniformMatrix4fv(mvp_id, 1, GL_FALSE, &model_view_projection[0][0]);

    mesh.render(shader_program);

    check_gl_error();

    glFlush();
  }

  /** Render the scene, calculating the view and inverse model matrices from attitudes (as quaternions) and translations.
   *
   * @param[in] the GLSL shader program.
   * @param[in] the field of view of the sensor.
   * @param[in] client object attitude.
   * @param[in] translation between the model and the sensor.
   * @param[in] sensor attitude.
   */
  void render(Shader* shader_program, const glm::dquat& model_q, const glm::dvec3& translate, const glm::dquat& camera_q) {
    glm::dquat flip = glm::angleAxis<double>(M_PI, glm::dvec3(0.0,1.0,0.0));
    
    glm::mat4 view          = glm::mat4(get_view_matrix(translate, camera_q));
    glm::mat4 inverse_model = glm::mat4(glm::mat4_cast(model_q * flip));

    render(shader_program, inverse_model, view);
  }


  /** Write the pose information to a file.
   *
   * @param[in] file basename (without the extension).
   * @param[in] the pose information to write to the file.
   */
  void save_pose_metadata(const std::string& basename, const glm::dmat4& pose) {
    std::string filename = basename + ".transform";
    std::ofstream out(filename.c_str());
    out << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1)
        << pose[0][0] << ' ' << pose[1][0] << ' ' << pose[2][0] << ' ' << pose[3][0] << '\n'
        << pose[0][1] << ' ' << pose[1][1] << ' ' << pose[2][1] << ' ' << pose[3][1] << '\n'
        << pose[0][2] << ' ' << pose[1][2] << ' ' << pose[2][2] << ' ' << pose[3][2] << '\n'
        << pose[0][3] << ' ' << pose[1][3] << ' ' << pose[2][3] << ' ' << pose[3][3] << std::endl;
    out.close();
  }


  /** Write the translation and rotation information to a file.
   *
   * @param[in] file basename (without the extension).
   * @param[in] model attitude quaternion.
   * @param[in] model-sensor translation.
   * @param[in] sensor attitude quaternion.
   */
  void save_transformation_metadata(const std::string& basename, const glm::dquat& model_q, const glm::dvec3& translate, const glm::dquat& camera_q) {
    save_pose_metadata(basename, get_model_view_matrix_without_scaling(model_q, translate, camera_q));
  }


  /** Return the transformation metadata as a 4x4 homogeneous matrix (float).
   *
   * @param[in] model attitude quaternion.
   * @param[in] model-sensor translation.
   * @param[in] sensor attitude quaternion.
   */
  glm::mat4 get_pose(const glm::dquat& model_q, const glm::dvec3& translate, const glm::dquat& camera_q) {
    return glm::mat4(get_model_view_matrix_without_scaling(model_q, translate, camera_q));
  }
  

  /** Gets the model view matrix without the scaling component.
   *
   * Used for unprojecting when we get our point cloud out.
   *
   * @param[in] model rotation quaternion.
   * @param[in] model-sensor translation.
   * @param[in] sensor rotation quaternion.
   *
   * \returns a 4x4 homogeneous transformation (the model-view matrix).
   */
  glm::dmat4 get_model_view_matrix_without_scaling(const glm::dquat& model, const glm::dvec3& translate, const glm::dquat& camera) {
    glm::dquat flip = glm::angleAxis<double>(M_PI, glm::dvec3(0.0,1.0,0.0));
    return get_view_matrix(translate, camera) * glm::mat4_cast(model * flip);
  }

  
  /** Gets the model view matrix.
   *
   * @param[in] model rotation quaternion.
   * @param[in] model-sensor translation.
   * @param[in] sensor rotation quaternion.
   *
   * \returns a 4x4 homogeneous transformation (the model-view matrix).
   */  
  glm::dmat4 get_model_view_matrix(const glm::dquat& model, const glm::dvec3& translate, const glm::dquat& camera) {
    return get_view_matrix(translate, camera) * get_model_matrix(model);
  }

  /** Gets the view matrix.
   *
   * @param[in] model-sensor translation.
   * @param[in] sensor rotation quaternion.
   *
   * \returns a 4x4 homogeneous transformation (the model-view matrix).
   */  
  glm::dmat4 get_view_matrix(const glm::dvec3& translate, const glm::dquat& camera_q) {
    
    // Need to ensure that translate is 0,0,-z, and adjust the camera rotation to compensate.
    glm::dvec3 negative_z(0.0,0.0,-1.0);
    double rotation_angle = std::acos(glm::dot(negative_z, glm::normalize(translate)));
    glm::dvec3 rotation_axis = glm::cross(negative_z, translate);
    double rotation_axis_length = glm::length(rotation_axis);
    if (rotation_axis_length == 0)
      rotation_axis = glm::dvec3(0.0, 1.0, 0.0);
    else
      rotation_axis = rotation_axis / rotation_axis_length;
    
    glm::dquat rotation = glm::angleAxis<double>(-rotation_angle, rotation_axis);

    glm::dquat y_flip = glm::angleAxis<double>(M_PI, glm::dvec3(0.0,1.0,0.0));
    glm::dvec3 adjusted_translate = glm::mat3_cast(rotation) * translate;
    glm::dquat adjusted_camera_q = camera_q * y_flip * rotation;
    
    return glm::mat4_cast(adjusted_camera_q) * glm::translate(glm::dmat4(1.0), adjusted_translate);
  }


  /** Gets the model transformation matrix.
   *
   * @param[in] model attitude quaternion.
   *
   * \returns a 4x4 homogeneous transformation (the model matrix).
   */  
  glm::dmat4 get_model_matrix(const glm::dquat& model) {
    glm::dquat flip = glm::angleAxis<double>(M_PI, glm::dvec3(0.0,1.0,0.0));
    return glm::mat4_cast(flip * model) * glm::scale(glm::dmat4(1.0), glm::dvec3(scale_factor, scale_factor, scale_factor));
  }
  

  float get_near_plane() const { return real_near_plane; }
  float get_far_plane() const { return far_plane; }


  double projected_area(unsigned int width, unsigned int height) const 
  {
    glm::ivec4 viewport;
    glm::mat4 identity(1.0);

    glGetIntegerv( GL_VIEWPORT, (int*)&viewport );

    unsigned char red[width*height];
    glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, (GLvoid*)(red));

    double area = 0.0;

    for (size_t ii = 0; ii < width * height; ++ii) {
      area += (double)(red[ii]) / 255.0;
    }
    
    return area;
  }
  

private:
  Mesh mesh;
  float scale_factor;

  glm::mat4 projection;
  float camera_d;
  GLfloat near_plane_bound;
  GLfloat real_near_plane;
  GLfloat far_plane;
  float box_width;          /* width of the orthographic projection box in
                             * object coordinates */
};

#endif // SCENE_HPP
