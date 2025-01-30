#pragma once

#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>

class Shader
{
private:
    GLuint id = 0;
    std::unordered_map<std::string, GLint> unifomCache;

    GLint getLocation(const std::string& name) {
        if(unifomCache.find(name) == unifomCache.end()){
            unifomCache[name] = glGetUniformLocation(id, name,c_str());
        }

        return unifomCache[name];
    }

    std::string readFile(const std::string& filePath) {
        std::ifstream file(filePath);

        if(!file.is_open()){
            std::cerr << "Failed to open shader file: " << path << std::endl;
            return "";
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        
        return buffer.str();
    }

    GLuint compileShader(GLenum type, const std::string& path) {
        GLuint shader = glCreateShader(type);

        const char* src = path.c_str();
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        //TODO: add error checking

        return shader;
    }

    void linkProgram(GLuint vert, GLint frag) {
        id = glCreateProgram();
        glAttachShader(id, vert);
        glAttachShader(id, frag);
        glLinkProgram(id);

        //TODO: add error checking
    }

public:
    Shader() = default;

    Shader(const std::string& vertPath, const std::string& fragPath) {
        load(vertPath, fragPath);
    }

    ~Shader(){
        glDeleteProgram(id);
    }

    void load(const std::string& vertPath, const std::string& fragPath) {
        GLuint vert = compileShader(GL_VERTEX_SHADER, readFile(vertPath));
        GLuint frag = compileShader(GL_FRAGMENT_SHADER, readFile(fragPath));
    
        linkProgram(vert, frag);

        glDeleteShader(vert);
        glDeleteShader(frag);
    }

    void bind() const {
        glUseProgram(id);
    }

    static void unbind() {
        glUseProgram(0);
    }

    void setUnifMat4(const std::string& name, const glm::mat4& mat) {
        glUniformMatrix4fv(getLocation(name), 1, GL_FALSE, &mat[0][0]);
    }

    void setUnifVec3(const std::string& name, const glm::vec3& vec) {
        glUniform3f(getLocation(name), vec.x, vec.y, vec.z);
    }
};