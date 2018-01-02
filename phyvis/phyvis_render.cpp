#include "phyvis_render.h"
#include <iostream>

namespace Phyvis {
    namespace Render {
        void Init(Context* context) {
            context->lineCount = 0;
            context->lineVAttribCount = 0;

            context->triangleCount = 0;
            context->triangleVAttribCount = 0;
            
            static const GLchar* vertexShaderSource = "#version 330 core\n"
                    "layout (location = 0) in vec2 inputPosition;\n"
                    "layout (location = 1) in vec3 inputColor;\n"
                    "uniform mat4 projection;"
                    "out vec3 colorFromVertex;\n"
                    "void main() {\n"
                    "   gl_Position = projection * vec4(inputPosition, 0.0f, 1.0f);\n"
                    "   colorFromVertex = inputColor;\n"
                    "}\n\0";
        
            static const GLchar* fragmentShaderSource = "#version 330 core\n"
                    "in vec3 colorFromVertex;\n"
                    "out vec4 color;\n"
                    "void main() {\n"
                    "   color = vec4(colorFromVertex, 1.0f);\n"
                    "}\n\0";
        
            GLchar infoLog[1024];
            GLint success;
        
            GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
            glCompileShader(vertexShader);
            glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
            if(!success)
            {
                glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
                std::cout<<infoLog<<std::endl;
            } else {
                std::cout<<"Success compiling vertex shader."<<std::endl;
            }
        
            GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
            glCompileShader(fragmentShader);
            glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
            if(!success)
            {
                glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
                std::cout<<infoLog<<std::endl;
            } else {
                std::cout<<"Success compiling fragment shader."<<std::endl;
            }
        
            context->shaderProgram = glCreateProgram();
            glAttachShader(context->shaderProgram, vertexShader);
            glAttachShader(context->shaderProgram, fragmentShader);
            glLinkProgram(context->shaderProgram);
            glGetShaderiv(context->shaderProgram, GL_LINK_STATUS, &success);
            if(!success)
            {
                glGetShaderInfoLog(context->shaderProgram, 512, NULL, infoLog);
                std::cout<<infoLog<<std::endl;
            } else {
                std::cout<<"Success linking shader program."<<std::endl;
            }
        
            glDeleteShader(vertexShader);
            glDeleteShader(fragmentShader);
        
            glGenVertexArrays(1, &context->VAO);
            glBindVertexArray(context->VAO);
        
            glGenBuffers(1, &context->VBO);
            glBindBuffer(GL_ARRAY_BUFFER, context->VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(context->lines), context->lines, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*) 0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*) (2 * sizeof(GLfloat))); 
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        
            glBindVertexArray(0);
        }

        void SetProjection(Context* context, float* projectionMat) {
            glUseProgram(context->shaderProgram);
            GLint projectionLocation = glGetUniformLocation(context->shaderProgram, "projection");
            glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, projectionMat);
        }

        void DrawLine(Context* context, ftVector2 p1, ftVector2 p2, Color color) {
            if (context->lineVAttribCount + 10 > BUFFER_SIZE) Render(context);
            context->lines[context->lineVAttribCount++] = p1.x;
            context->lines[context->lineVAttribCount++] = p1.y;
            context->lines[context->lineVAttribCount++] = color.r;
            context->lines[context->lineVAttribCount++] = color.g;
            context->lines[context->lineVAttribCount++] = color.b;
            
            context->lines[context->lineVAttribCount++] = p2.x;
            context->lines[context->lineVAttribCount++] = p2.y;
            context->lines[context->lineVAttribCount++] = color.r;
            context->lines[context->lineVAttribCount++] = color.g;
            context->lines[context->lineVAttribCount++] = color.b;
        
            ++context->lineCount;
        }

        void DrawSolidTriangle(Context* context, 
                            ftVector2 p1, 
                            ftVector2 p2, 
                            ftVector2 p3, 
                            Color color) {
            if (context->triangleVAttribCount + 15 > BUFFER_SIZE) Render(context);
            context->triangles[context->triangleVAttribCount++] = p1.x;
            context->triangles[context->triangleVAttribCount++] = p1.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
        
            context->triangles[context->triangleVAttribCount++] = p2.x;
            context->triangles[context->triangleVAttribCount++] = p2.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
        
            context->triangles[context->triangleVAttribCount++] = p3.x;
            context->triangles[context->triangleVAttribCount++] = p3.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
        
            ++context->triangleCount;
        }
    
        void DrawOutlinePolygon(Context* context, 
                            ftVector2* vertexes, 
                            int vertexCount, 
                            Color color) {
            ftVector2 vert1 = vertexes[vertexCount - 1];
            for (int i = 0; i < vertexCount; ++i) {
                ftVector2 vert2 = vertexes[i];
                DrawLine(context, vert1, vert2, color);
                vert1 = vert2;
            }
        }
    
        void DrawOutlineCircle(Context* context, 
                            ftVector2 center, 
                            real radius, 
                            Color color) {
            int numSegment = 20;
            
            ftVector2 p1 = {radius, 0};
            float tFrac = tanf( 2 * 3.14159265 / numSegment);
            float rFrac = cosf( 2 * 3.14159265 / numSegment);
            
            for (int i = 0; i < numSegment; ++i) {
                ftVector2 t1 = {p1.y , - p1.x};
                ftVector2 p2  = (p1 + t1 * tFrac) * rFrac;
                DrawLine(context, center + p1, center + p2, color);
                p1 = p2;
            }
        }
        
        void DrawSolidCircle(Context* context,
                            ftVector2 center, 
                            real radius, 
                            Color color) {
            int numSegment = 20;
            
            ftVector2 p1 = {0, radius};
            float tFrac = tanf( 2 * 3.14159265 / numSegment);
            float rFrac = cosf( 2 * 3.14159265 / numSegment);
            
            for (int i = 0; i < numSegment; ++i) {
                ftVector2 t1 = {p1.y , - p1.x};
                ftVector2 p2  = (p1 + t1 * tFrac) * rFrac;
                DrawSolidTriangle(context, center, center + p1, center + p2, color);
                p1 = p2;
            }
        } 
        
        void DrawOutlineRect(Context* context, 
                        ftVector2 corner1, 
                        ftVector2 corner2, 
                        Color color) {
    
            DrawLine(context, corner1, {corner1.x, corner2.y}, color);
            DrawLine(context, corner1, {corner2.x, corner1.y}, color);
            DrawLine(context, corner2, {corner1.x, corner2.y}, color);
            DrawLine(context, corner2, {corner2.x, corner1.y}, color);
            
        }

        void Render(Context* context) {
            
            glUseProgram(context->shaderProgram);
            glBindVertexArray(context->VAO);
        
            glBindBuffer(GL_ARRAY_BUFFER, context->VBO);
        
            glBufferSubData(GL_ARRAY_BUFFER, 0, context->lineVAttribCount * sizeof(GLfloat), context->lines);
            glDrawArrays(GL_LINES, 0, context->lineCount * 2);
        
            glBufferSubData(GL_ARRAY_BUFFER, 0, context->triangleVAttribCount * sizeof(GLfloat), context->triangles);
            glDrawArrays(GL_TRIANGLES, 0, context->triangleCount * 3);
        
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
            glUseProgram(0);
        
            context->lineCount = 0;
            context->lineVAttribCount = 0;
        
            context->triangleCount = 0;
            context->triangleVAttribCount = 0;
        }
            
        void Cleanup(Context* context) {
            glDeleteProgram(context->shaderProgram);
            glDeleteVertexArrays(1, &context->VAO);
            glDeleteBuffers(1, &context->VBO);
        }
    }
}