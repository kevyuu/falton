#ifndef FALTON_DRAW_H
#define FALTON_DRAW_H

#include <falton/shape/ftPolygon.h>
#include <falton/shape/ftCircle.h>
#include <falton/setting.h>
#include <iostream>
#include <cmath>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

struct Color {

    float r, g, b;

    void setRGB(uint8 inputR, uint8 inputG, uint8 inputB) {
        r = (float) inputR / 255;
        g = (float) inputG / 255;
        b = (float) inputB / 255;
    }
};

struct ftDebugCamera {
    ftVector2 center;
    float halfWidth, halfHeight;

    void MoveRight();
    void MoveLeft();
    void MoveUp();
    void MoveDown();
    void ZoomIn();
    void ZoomOut();
};

class ftDebugCanvas {
public:
    void Init();
    void SetCamera(const ftDebugCamera& camera);
    void DrawLine(const ftVector2& p1, const ftVector2& p2, const Color& color);
    void DrawSolidTriangle(const ftVector2&p1, const ftVector2& p2, const ftVector2& p3, const Color& color);
    void DrawPolygon(const ftPolygon& polygon, const ftTransform& transform, const Color& color);
    void DrawCircle(const ftCircle& circle, const ftTransform& transform, const Color& color);
    void DrawSolidCircle(const ftVector2& center, real radius, const Color& color);
    void DrawBody(ftBody* body, Color color);
    void DrawPhysics(ftPhysicsSystem* physicsSystem);
    void Render();
    void Cleanup();
private:
    
    static constexpr int BUFFER_SIZE = 1024;
    GLfloat lines[BUFFER_SIZE];
    uint32 nLine;
    uint32 nLineVAttrib;

    GLfloat triangles[BUFFER_SIZE];
    uint32 nTriangle;
    uint32 nTriangleVAttrib;

    GLuint VBO;
    GLuint VAO;
    GLuint shaderProgram;
};

#endif