#pragma once

// The including translation unit must include the OpenGL headers before this file.

#include <iostream>
#include <string>

namespace demdl {

// Compiles and links a vertex/fragment pair from source strings. Trimmed from the LearnOpenGL
// helper: no file loading, no GLM, only the one uniform setter the preview needs.
class Shader {
 public:
  Shader(const std::string& vertexSource, const std::string& fragmentSource) {
    const unsigned int vertex = compile(GL_VERTEX_SHADER, vertexSource, "vertex");
    const unsigned int fragment = compile(GL_FRAGMENT_SHADER, fragmentSource, "fragment");
    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    glLinkProgram(id_);
    int success = 0;
    glGetProgramiv(id_, GL_LINK_STATUS, &success);
    if (!success) {
      char log[1024];
      glGetProgramInfoLog(id_, sizeof log, nullptr, log);
      std::cerr << "shader link error: " << log << "\n";
    }
    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }
  ~Shader() {
    if (id_ != 0) glDeleteProgram(id_);
  }
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  void use() const { glUseProgram(id_); }
  void setInt(const char* name, int value) const { glUniform1i(glGetUniformLocation(id_, name), value); }

 private:
  static unsigned int compile(unsigned int type, const std::string& source, const char* label) {
    const unsigned int shader = glCreateShader(type);
    const char* text = source.c_str();
    glShaderSource(shader, 1, &text, nullptr);
    glCompileShader(shader);
    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char log[1024];
      glGetShaderInfoLog(shader, sizeof log, nullptr, log);
      std::cerr << label << " shader compile error: " << log << "\n";
    }
    return shader;
  }

  unsigned int id_ = 0;
};

}  // namespace demdl
