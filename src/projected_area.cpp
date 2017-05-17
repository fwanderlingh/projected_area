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
#include <cstdio>
#include <list>

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



/** @brief Read quaternions from some input stream
 *
 * @param[in,out]  in  input stream to read from
 * @param[out]     q   quaternion where result is stored, normalized
 *
 * @returns true if read was successful; false if EOF was encountered at first read.
 */
bool read_quaternion(std::istream& in, glm::dquat& q) 
{
  double w, x, y, z;

  if (in.peek() == 'x' || in.fail()) return false;
  
  in >> w;  
  in >> x;
  in >> y;
  in >> z;

  if (in.fail()) return false;
  
  
  q = glm::normalize(glm::dquat(w, x, y, z));
  return true;
}


/** @brief Read a list of quaternions from some input stream (until EOF)
 *
 * @param[in,out]  in   input stream to read from
 *
 * @returns a linked list of quaternions
 */
std::list<glm::dquat> read_quaternions(std::istream& in) 
{
  std::list<glm::dquat> l;
  glm::dquat q;
  bool succ = read_quaternion(in, q);

  while (succ) {
    l.push_back(q);
    succ = read_quaternion(in, q);
  }

  return l;  
}


/** @brief Read a list of quaternions from a file
 *
 * @param[in]  filename   input filename
 *
 * @returns a linked list of quaternions
 */
std::list<glm::dquat> read_quaternions(const std::string& filename) 
{
  std::ifstream f(filename.c_str());
  return read_quaternions(f);
}


/** @brief Write a list of doubles to an output stream
 *
 * @param[in,out] out     output stream
 * @param[in]     areas   list of doubles to write
 */
void write_projected_areas(std::ostream& out, const std::list<double>& areas)
{
  for (std::list<double>::const_iterator it = areas.begin(); it != areas.end(); ++it) {
    out << *it << std::endl;
  }
}


/** @brief Write a list of doubles to a file
 *
 * @param[in]  filename  where to write
 * @param[in]  areas     list of doubles to write
 */
void write_projected_areas(const std::string& filename, const std::list<double>& areas) 
{
  std::ofstream f(filename.c_str());
  write_projected_areas(f, areas);
}



std::ostream& operator<<(std::ostream& out, const glm::dquat& q) 
{
  out << '[' << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ']' << std::flush;
  return out;
}



/** @brief Main function
 **
 * @detail Sets everything up and then loops --- pretty standard OpenGL --- to
 *         intercept keypresses, mouseclicks, to modify the scene, and to redraw.
 *
 * @param[in] argc  Number of arguments from the command line.
 * @param[in] argv  Array of character pointers to the command line arguments.
 *
 * \returns An integer describing the exit status.
 */
int main(int argc, char** argv) {

  std::string model_filename;
  if (argc < 3) {
    std::cerr << "Usage: projected_area filename maximum_dimension [window_size [-p]]" << std::endl;
    return -1;
  }
  float box_width = 10.0;
  unsigned int width = 1024, height = 1024;
  
  model_filename   = argv[1];
  box_width        = atof(argv[2]);
  
  if (argc > 3)
    width = height = atoi(argv[3]);
  
  bool persist = false;
  if (argc > 4)
    persist = (argv[4][1] == 'p');
  
  float model_scale_factor = 1.0;
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
  glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);

  // std::cerr << "OpenGL version " << glGetString(GL_VERSION) << std::endl;

  /*
   * 4. Attempt to create a window that we can draw our LIDAR images in.
   */
  GLFWwindow* window = glfwCreateWindow(width, height, "Projected Area", NULL, NULL);
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
  //glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

  Scene  scene(model_filename, model_scale_factor, -translation[2], box_width);
  Shader shader_program("shaders/vertex.glsl", "shaders/fragment.glsl");
  
  //glfwSwapBuffers(window);
  
  
  /*
   * Main event loop
   */

  glm::dquat object;
  bool succ = true;
  size_t count = 0;
  
  while (succ && !s_interrupted && !std::cin.eof() && !std::cin.fail()) {

    succ = read_quaternion(std::cin, object);

    scene.render(&shader_program, object, translation, sensor);
    //glfwSwapBuffers(window);
    
    double pixels = scene.projected_area(width, height);
    double projected_area = pixels * box_width * box_width / (double)(width * height);
    
    std::cout << std::setprecision(17)
              << object.w << ' '
              << object.x << ' '
              << object.y << ' '
              << object.z << ' '
              << projected_area << std::endl;

    ++count;
    
  }
  std::cerr << "Done printing areas for " << count << " attitudes." << std::endl;

  while (persist && !s_interrupted) {
    scene.render(&shader_program, object, translation, sensor);
  }
  

  glfwTerminate();
  

  return 0;
}
