#include "ftDebugCanvas.h"

void ftDebugCamera::MoveUp() {
    center.y += halfHeight / 100;
}

void ftDebugCamera::MoveDown() {
    center.y -= halfHeight / 100;
}

void ftDebugCamera::MoveRight() {
    center.x += halfWidth / 100;
}

void ftDebugCamera::MoveLeft() {
    center.x -= halfWidth / 100;
}

void ftDebugCamera::ZoomIn() {
    halfWidth *= 0.95f;
    halfHeight *= 0.95f;
}

void ftDebugCamera::ZoomOut() {
    halfWidth *= 1.05f;
    halfHeight *= 1.05f;
}

void ftDebugCanvas::Init() {

	nLine = 0;
	nLineVAttrib = 0;

    nTriangle = 0;
    nTriangleVAttrib = 0;

    const GLchar* vertexShaderSource = "#version 330 core\n"
            "layout (location = 0) in vec2 inputPosition;\n"
            "layout (location = 1) in vec3 inputColor;\n"
            "uniform mat4 projection;"
            "out vec3 colorFromVertex;\n"
            "void main() {\n"
            "   gl_Position = projection * vec4(inputPosition, 0.0f, 1.0f);\n"
            "   colorFromVertex = inputColor;\n"
            "}\n\0";

    const GLchar* fragmentShaderSource = "#version 330 core\n"
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

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetShaderiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout<<infoLog<<std::endl;
    } else {
        std::cout<<"Success linking shader program."<<std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(lines), lines, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*) 0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void*) (2 * sizeof(GLfloat))); 
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);

}

void ftDebugCanvas::SetCamera(const ftDebugCamera& camera) {

    float centerX = camera.center.x;
    float centerY = camera.center.y;
    float halfWidth = camera.halfWidth;
    float halfHeight = camera.halfHeight;
    float l = centerX - halfWidth;
    float r = centerX + halfWidth;
    float b = centerY - halfHeight;
    float t = centerY + halfHeight;
    GLfloat projectionMat[] = {
        2.0f / (r - l), 0.0f, 0.0f, 0.0f,
        0.0f, 2.0f / (t - b), 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        - (r + l) / (r - l), - (t + b) / (t - b), 0.0f, 1.0f
    };

    glUseProgram(shaderProgram);
    GLint projectionLocation = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, projectionMat);
    
}

void ftDebugCanvas::DrawLine(const ftVector2& p1, const ftVector2& p2, const Color& color) {
    if (nLineVAttrib + 10 > BUFFER_SIZE) Render();
    lines[nLineVAttrib++] = p1.x;
    lines[nLineVAttrib++] = p1.y;
    lines[nLineVAttrib++] = color.r;
    lines[nLineVAttrib++] = color.g;
    lines[nLineVAttrib++] = color.b;
    
    lines[nLineVAttrib++] = p2.x;
    lines[nLineVAttrib++] = p2.y;
    lines[nLineVAttrib++] = color.r;
    lines[nLineVAttrib++] = color.g;
    lines[nLineVAttrib++] = color.b;

    ++nLine;
}

void ftDebugCanvas::DrawSolidTriangle(const ftVector2& p1, const ftVector2& p2, const ftVector2& p3, const Color& color) {
    if (nTriangleVAttrib + 15 > BUFFER_SIZE) Render();
    triangles[nTriangleVAttrib++] = p1.x;
    triangles[nTriangleVAttrib++] = p1.y;
    triangles[nTriangleVAttrib++] = color.r;
    triangles[nTriangleVAttrib++] = color.g;
    triangles[nTriangleVAttrib++] = color.b;

    triangles[nTriangleVAttrib++] = p2.x;
    triangles[nTriangleVAttrib++] = p2.y;
    triangles[nTriangleVAttrib++] = color.r;
    triangles[nTriangleVAttrib++] = color.g;
    triangles[nTriangleVAttrib++] = color.b;

    triangles[nTriangleVAttrib++] = p3.x;
    triangles[nTriangleVAttrib++] = p3.y;
    triangles[nTriangleVAttrib++] = color.r;
    triangles[nTriangleVAttrib++] = color.g;
    triangles[nTriangleVAttrib++] = color.b;

    ++nTriangle;
}

void ftDebugCanvas::DrawPolygon(const ftPolygon& polygon, const ftTransform& transform, const Color& color) {
    ftVector2 vert1 = transform * polygon.vertices[polygon.numVertex - 1];
    for (uint32 i = 0; i < polygon.numVertex; ++i) {
        ftVector2 vert2 = transform * polygon.vertices[i];
        DrawLine(vert1, vert2, color);
        vert1 = vert2;
    }
}

void ftDebugCanvas::DrawCircle(const ftCircle& circle, const ftTransform& transform, const Color& color) {
    int numSegment = 20;
    
    ftVector2 p1 = {0, circle.radius};
    float tFrac = tanf( 2 * 3.14159265 / numSegment);
    float rFrac = cosf( 2 * 3.14159265 / numSegment);

    ftVector2 radialPoint = transform.rotation * (p1);
    DrawLine(transform.center, transform.center + radialPoint, color);
    
    for (uint32 i = 0; i < numSegment; ++i) {
        ftVector2 t1 = {p1.y , - p1.x};
        ftVector2 p2  = (p1 + t1 * tFrac) * rFrac;
        DrawLine(transform.center + p1, transform.center + p2, color);
        p1 = p2;
    }
}

void ftDebugCanvas::DrawSolidCircle(const ftVector2& center, real radius, const Color& color) {
    int numSegment = 20;
    
    ftVector2 p1 = {0, radius};
    float tFrac = tanf( 2 * 3.14159265 / numSegment);
    float rFrac = cosf( 2 * 3.14159265 / numSegment);
    
    for (uint32 i = 0; i < numSegment; ++i) {
        ftVector2 t1 = {p1.y , - p1.x};
        ftVector2 p2  = (p1 + t1 * tFrac) * rFrac;
        DrawSolidTriangle(center, center + p1, center + p2, color);
        p1 = p2;
    }
}


void ftDebugCanvas::DrawBody(const ftBody* const body, Color color) {
    for (ftCollider* collider = body->colliders; collider != nullptr; collider = collider->next) {
        if (collider->shape->shapeType == SHAPE_CIRCLE) {
            DrawCircle(*((ftCircle*) collider->shape), body->transform, color);
        } else {
            DrawPolygon(*((ftPolygon*) collider->shape), body->transform, color);
        }
    }
}


void ftDebugCanvas::DrawPhysics(ftPhysicsSystem* physicsSystem) {
    ftDebugCanvas* pCanvas = this;
    const auto drawStatic = [pCanvas] (ftBody* body) {
        Color redColor = {1.0f, 0.0f, 0.0f};
        pCanvas->DrawBody(body, redColor);
    };

    const auto drawKinematic = [pCanvas] (ftBody* body) {
        Color greenColor = {0.0f, 1.0f, 0.0f};
        pCanvas->DrawBody(body, greenColor);
    };

    const auto drawDynamic = [pCanvas] (ftBody* body) {
        Color blueColor = {0.0f, 0.0f, 1.0f};
        pCanvas->DrawBody(body, blueColor);
    };

    const auto drawContact = [pCanvas] (ftContact* contact) {
        Color contactColor = {0.2f, 0.8f, 0.3f};
        for (int i =0 ; i < contact->manifold.numContact; ++i) {
            pCanvas->DrawSolidCircle(contact->manifold.contactPoints[0].r1, 0.1, contactColor);
            pCanvas->DrawSolidCircle(contact->manifold.contactPoints[0].r2, 0.1, contactColor);
        }
    };

    physicsSystem->forEachStaticBody(drawStatic);
    physicsSystem->forEachKinematicBody(drawKinematic);
    physicsSystem->forEachDynamicBody(drawDynamic);
    physicsSystem->forEachContact(drawContact);
}

void ftDebugCanvas::Render() {

    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferSubData(GL_ARRAY_BUFFER, 0, nLineVAttrib * sizeof(GLfloat), lines);
    glDrawArrays(GL_LINES, 0, nLine * 2);

    glBufferSubData(GL_ARRAY_BUFFER, 0, nTriangleVAttrib * sizeof(GLfloat), triangles);
    glDrawArrays(GL_TRIANGLES, 0, nTriangle * 3);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);

    nLine = 0;
    nLineVAttrib = 0;

    nTriangle = 0;
    nTriangleVAttrib = 0;
}

void ftDebugCanvas::Cleanup() {
    glDeleteProgram(shaderProgram);
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}