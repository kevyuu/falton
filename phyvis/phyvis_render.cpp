#include "phyvis_render.h"
#include <iostream>

namespace Phyvis {
    namespace Render {
        void Init(Context* context) {
            context->lineCount = 0;
            context->lineVAttribCount = 0;

            context->triangleCount = 0;
            context->triangleVAttribCount = 0;

            context->transformCount = 0;
            
            static const GLchar* vertexShaderSource = "#version 330 core\n"
                    "layout (location = 0) in vec2 inputPosition;\n"
                    "layout (location = 1) in vec4 inputColor;\n"
                    "uniform mat4 projection;"
                    "out vec4 colorFromVertex;\n"
                    "void main() {\n"
                    "   gl_Position = projection * vec4(inputPosition, 0.0f, 1.0f);\n"
                    "   colorFromVertex = inputColor;\n"
                    "}\n\0";
        
            static const GLchar* fragmentShaderSource = "#version 330 core\n"
                    "in vec4 colorFromVertex;\n"
                    "out vec4 color;\n"
                    "void main() {\n"
                    "   color = colorFromVertex;\n"
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
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*) 0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*) (2 * sizeof(GLfloat))); 
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

        void PushTransform(Context* context, const ftTransform& transform) {
            context->transform[context->transformCount] = transform;
            ++context->transformCount;
        }

        void PopTransform(Context* context) {
            --context->transformCount;
        }

        void DrawLine(Context* context, ftVector2 p1, ftVector2 p2, Color color) {
            if (context->lineVAttribCount + 10 > BUFFER_SIZE) Render(context);
            for (int i = 0; i < context->transformCount; ++i) {
                p1 = context->transform[i] * p1;
                p2 = context->transform[i] * p2;
            }
            context->lines[context->lineVAttribCount++] = p1.x;
            context->lines[context->lineVAttribCount++] = p1.y;
            context->lines[context->lineVAttribCount++] = color.r;
            context->lines[context->lineVAttribCount++] = color.g;
            context->lines[context->lineVAttribCount++] = color.b;
            context->lines[context->lineVAttribCount++] = color.a;
            
            context->lines[context->lineVAttribCount++] = p2.x;
            context->lines[context->lineVAttribCount++] = p2.y;
            context->lines[context->lineVAttribCount++] = color.r;
            context->lines[context->lineVAttribCount++] = color.g;
            context->lines[context->lineVAttribCount++] = color.b;
            context->lines[context->lineVAttribCount++] = color.a;
        
            ++context->lineCount;
        }

        void DrawDashedLine(Context* context, ftVector2 p1, ftVector2 p2, Color color) {
            static const int NUMBER_OF_DASHED_LINE = 10;
            ftVector2 lineDim = (p2 - p1) / (NUMBER_OF_DASHED_LINE * 2);
            for (int i = 0; i < 10; ++i) {
                ftVector2 start = p1 + (lineDim * i * 2);
                ftVector2 end = p1 + (lineDim * ((i * 2) + 1));
                DrawLine(context, start, end, color);
            }
        }

		void DrawDashedLine(Context* context, ftVector2 p1, ftVector2 p2, float dashLength, Color color) {
			float diffX = p2.x - p1.x;
			float diffY = p2.y - p1.y;
			float dist = sqrt(diffX * diffX + diffY * diffY);
			int dashLineCount = dist / (dashLength * 2);
			ftVector2 lineDim = (p2 - p1) / (dashLineCount * 2);
			for (int i = 0; i < dashLineCount; ++i) {
				ftVector2 start = p1 + (lineDim * i * 2);
				ftVector2 end = p1 + (lineDim * ((i * 2) + 1));
				DrawLine(context, start, end, color);
			}
        }
        
        void DrawOutlineTriangle(Context* context,
            ftVector2 p1,
            ftVector2 p2,
            ftVector2 p3,
            Color color) {
            
            DrawLine(context, p1, p2, color);
            DrawLine(context, p2, p3, color);
            DrawLine(context, p3, p1, color);
        }

        void DrawSolidTriangle(Context* context, 
                            ftVector2 p1, 
                            ftVector2 p2, 
                            ftVector2 p3, 
                            Color color) {
            if (context->triangleVAttribCount + 18 > BUFFER_SIZE) Render(context);

            for (int i = 0; i < context->transformCount; ++i) {
                p1 = context->transform[i] * p1;
                p2 = context->transform[i] * p2;
                p3 = context->transform[i] * p3;
            }

            context->triangles[context->triangleVAttribCount++] = p1.x;
            context->triangles[context->triangleVAttribCount++] = p1.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
            context->triangles[context->triangleVAttribCount++] = color.a;
        
            context->triangles[context->triangleVAttribCount++] = p2.x;
            context->triangles[context->triangleVAttribCount++] = p2.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
            context->triangles[context->triangleVAttribCount++] = color.a;
        
            context->triangles[context->triangleVAttribCount++] = p3.x;
            context->triangles[context->triangleVAttribCount++] = p3.y;
            context->triangles[context->triangleVAttribCount++] = color.r;
            context->triangles[context->triangleVAttribCount++] = color.g;
            context->triangles[context->triangleVAttribCount++] = color.b;
            context->triangles[context->triangleVAttribCount++] = color.a;
        
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

		void DrawSolidPolygon(Context* context,
			ftVector2* vertexes,
			int vertexCount,
			Color color) {

			ftVector2 p1 = vertexes[0];
			for (int i = 1; i < vertexCount - 1; ++i) {
				ftVector2 p2 = vertexes[i];
				ftVector2 p3 = vertexes[i + 1];
				DrawSolidTriangle(context, p1, p2, p3, color);
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

		void DrawOutlineQuad(Context* context,
			ftVector2 p1,
			ftVector2 p2,
			ftVector2 p3,
			ftVector2 p4,
			Color color) {
			DrawLine(context, p1, p2, color);
			DrawLine(context, p2, p3, color);
			DrawLine(context, p3, p4, color);
			DrawLine(context, p4, p1, color);
		}

		void DrawDashedRect(Context* context,
			ftVector2 corner1,
			ftVector2 corner2,
			float dashLength,
			Color color) {
			DrawDashedLine(context, corner1, { corner1.x, corner2.y }, dashLength, color);
			DrawDashedLine(context, corner1, { corner2.x, corner1.y }, dashLength, color);
			DrawDashedLine(context, corner2, { corner1.x, corner2.y }, dashLength, color);
			DrawDashedLine(context, corner2, { corner2.x, corner1.y }, dashLength, color);
		}

		void DrawSolidRect(Context* context,
			ftVector2 corner1,
			ftVector2 corner2,
			Color color) {
			DrawSolidTriangle(context, corner1, corner2, { corner1.x, corner2.y }, color);
			DrawSolidTriangle(context, corner1, corner2, { corner2.x, corner1.y }, color);
		}

        void DrawDashedRect(Context* context,
                            ftVector2 corner1, 
                            ftVector2 corner2,
                            Color color) {
            
            DrawDashedLine(context, corner1, {corner1.x, corner2.y}, color);
            DrawDashedLine(context, corner1, {corner2.x, corner1.y}, color);
            DrawDashedLine(context, corner2, {corner1.x, corner2.y}, color);
            DrawDashedLine(context, corner2, {corner2.x, corner1.y}, color);
        }

        void Render(Context* context) {
            
            glUseProgram(context->shaderProgram);
            glBindVertexArray(context->VAO);
        
            glBindBuffer(GL_ARRAY_BUFFER, context->VBO);
			
            glEnable(GL_LINE_SMOOTH);
            glEnable(GL_POLYGON_SMOOTH);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
            glDisable(GL_DEPTH_TEST);

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