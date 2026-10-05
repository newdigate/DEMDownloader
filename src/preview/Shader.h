#pragma once

// The including translation unit must include the OpenGL headers before this file.

#include <stdexcept>
#include <string>

namespace demdl {

// Compiles and links a vertex/fragment pair from source strings. Trimmed from the LearnOpenGL
// helper: no file loading, no GLM, only the one uniform setter the preview needs.
class Shader {
 public:
  // Throws std::runtime_error with the driver's log when a shader does not compile or the program
  // does not link, after deleting the shader and program objects it created.
  Shader(const std::string& vertexSource, const std::string& fragmentSource) {
    const unsigned int vertex = compile(GL_VERTEX_SHADER, vertexSource, "vertex");
    unsigned int fragment = 0;
    try {
      fragment = compile(GL_FRAGMENT_SHADER, fragmentSource, "fragment");
    } catch (...) {
      glDeleteShader(vertex);
      throw;
    }
    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    glLinkProgram(id_);
    // The program keeps what it linked, so the shader objects can go whether or not it linked.
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    int success = 0;
    glGetProgramiv(id_, GL_LINK_STATUS, &success);
    if (!success) {
      char log[1024] = "";
      glGetProgramInfoLog(id_, sizeof log, nullptr, log);
      glDeleteProgram(id_);  // the destructor does not run for a constructor that throws
      throw std::runtime_error("shader link error: " + tidy(log));
    }
  }
  ~Shader() {
    if (id_ != 0) glDeleteProgram(id_);
  }
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  void use() const { glUseProgram(id_); }
  void setInt(const char* name, int value) const { glUniform1i(glGetUniformLocation(id_, name), value); }

 private:
  // Throws, having deleted the shader, when the source does not compile.
  static unsigned int compile(unsigned int type, const std::string& source, const char* label) {
    const unsigned int shader = glCreateShader(type);
    const char* text = source.c_str();
    glShaderSource(shader, 1, &text, nullptr);
    glCompileShader(shader);
    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char log[1024] = "";
      glGetShaderInfoLog(shader, sizeof log, nullptr, log);
      glDeleteShader(shader);
      throw std::runtime_error(std::string(label) + " shader compile error: " + tidy(log));
    }
    return shader;
  }

  // The driver's log without its trailing line break.
  static std::string tidy(const char* log) {
    std::string text(log);
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
    return text;
  }

  unsigned int id_ = 0;
};

}  // namespace demdl
