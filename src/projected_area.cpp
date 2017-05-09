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
 ** @brief Compute the projected area of an object by rendering it using OpenGL.
 **        This source code is based on GLIDAR.
 **
 **/

#include <iostream>
#include <sstream>
#include <csignal>

#include "../include/gl_error.hpp"
#include "../include/scene.hpp"
#include "../include/mesh.hpp"

/*
 * These next few functions are for catching Ctrl+C interrupts.
 */
static int s_interrupted = 0;
static void s_signal_handler(int signal_value) {
  s_interrupted = 1;
}
static void s_catch_signals(void) {
  struct sigaction action;
  action.sa_handler = s_signal_handler;
  action.sa_flags = 0;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, NULL);
  sigaction(SIGTERM, &action, NULL);
}



/** @brief Main function
 **
 * @detail Sets everything up and then loops --- pretty standard OpenGL --- to
 *         intercept keypresses, mouseclicks, to modify the scene, and to redraw.
 *
 * @param[in] Number of arguments from the command line.
 * @param[in] Array of character pointers to the command line arguments.
 *
 * \returns An integer describing the exit status.
 */
int main(int argc, char** argv) {

  std::string model_filename;
  if (argc < 7) {
    std::cerr << "Usage: projected_area filename maximum_dimension qw qx qy qz [window_size]" << std::endl;
    exit(-1);
  }
  float box_width = 10.0;
  unsigned int width = 1024, height = 1024;
  
  model_filename   = argv[1];
  box_width        = atof(argv[2]);
  double qw, qx, qy, qz;
  qw               = atof(argv[3]);
  qx               = atof(argv[4]);
  qy               = atof(argv[5]);
  qz               = atof(argv[6]);

  std::cerr << "Read quaternion: " << qw << ' ' << qx << ' ' << qy << ' ' << qz << std::endl;
  
  if (argc > 7)
    width = height = atoi(argv[7]);

  float model_scale_factor = 1.0;
  glm::dquat object = glm::dquat(qw, qx, qy, qz);
  glm::dquat sensor(1.0, 0.0, 0.0, 0.0);
  glm::dvec3 translation(0.0, 0.0, 100.0);
  
  /* This is where we start to catch signals. */
  s_catch_signals ();

  std::cerr << "Loading model "      << model_filename << std::endl;
  std::cerr << "Scaling model by "   << model_scale_factor << std::endl;

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return -1;
  }

  glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2); // We want OpenGL 2.1 (latest that will work on my MBA's Intel Sandy Bridge GPU)
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  // std::cerr << "OpenGL version " << glGetString(GL_VERSION) << std::endl;

  /*
   * 4. Attempt to create a window that we can draw our LIDAR images in.
   */
  GLFWwindow* window = glfwCreateWindow(width, height, "GLIDAR", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to open GLFW window." << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);

  // Initialize GLEW
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return -1;
  }

  // Ensure we can capture keypresses.
  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

  Scene  scene(model_filename, model_scale_factor, -translation[2], box_width);
  Shader shader_program("shaders/vertex.glsl", "shaders/fragment.glsl");



  /*
   * Main event loop
   */
  do {
    scene.render(&shader_program, object, translation, sensor);    
    
    if (s_interrupted && glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwWindowShouldClose(window))  {
      std::cerr << "Exiting." << std::endl;

      double pixels = scene.projected_area(width, height);
      double projected_area = pixels * box_width * box_width / (double)(width * height);
      std::cout << std::setprecision(17) << projected_area << std::endl;
      s_interrupted = true;
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
  } while (!s_interrupted);

  glfwTerminate();
  

  return 0;
}
