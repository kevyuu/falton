#pragma once

#include <GL/gl3w.h>
#include "falton/math.h"

namespace Phyvis {
    namespace Render {

        struct Color {
            float r;
            float g;
            float b;
            float a;
        };

        static constexpr int BUFFER_SIZE = 1024;
        
        struct Context {
            GLfloat lines[BUFFER_SIZE];
            uint32 lineCount;
            uint32 lineVAttribCount;
        
            GLfloat triangles[BUFFER_SIZE];
            uint32 triangleCount;
            uint32 triangleVAttribCount;
        
            GLuint VBO;
            GLuint VAO;
            GLuint shaderProgram;

            ftTransform transform[20];
            int transformCount;
        };

        void Init(Context* context);
        void SetProjection(Context* context, float* projectionMat);
        void PushTransform(Context*context, const ftTransform& transform);
        void PopTransform(Context* context);

        void DrawLine(Context* context, ftVector2 p1, ftVector2 p2, Color color);
        void DrawDashedLine(Context* context, ftVector2 p1, ftVector2 p2, Color color);
        void DrawDashedLine(Context* context, ftVector2 p1, ftVector2 p2, float dashLength, Color color);
        void DrawOutlineTriangle(Context* context, ftVector2 p1, ftVector2 p2, ftVector2 p3, Color color);
        void DrawSolidTriangle(Context* context, ftVector2 p1, ftVector2 p2, ftVector2 p3, Color color);
        void DrawOutlinePolygon(Context* context, ftVector2 *vertexes, int vertexCount, Color color);
        void DrawSolidPolygon(Context* context, ftVector2* vertexes, int vertexCount, Color color);
        void DrawOutlineCircle(Context* context, ftVector2 center, real radius, Color color);
        void DrawSolidCircle(Context* context, ftVector2 center, real radius, Color color);
		void DrawOutlineQuad(Context* context, ftVector2 p1, ftVector2 p2, ftVector2 p3, ftVector2 p4, Color color);
        void DrawOutlineRect(Context* context, ftVector2 corner1, ftVector2 corner2, Color color);
        void DrawDashedRect(Context* context, ftVector2 corner1, ftVector2 corner2, Color color);
		void DrawSolidRect(Context* context, ftVector2 corner1, ftVector2 corner2, Color color);
		void DrawDashedRect(Context* context, ftVector2 corner1, ftVector2 corner2, float dashLength, Color color);
        void Render(Context* context);
        void Cleanup(Context* context);

    }
}