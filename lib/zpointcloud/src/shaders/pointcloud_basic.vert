#version 330 core

in vec3 vertexPosition;
in vec3 vertexColor;

out vec3 position;
out vec3 color;

uniform mat4 modelView;
uniform mat4 mvp;

uniform bool hasColors = false;
uniform vec3 defaultColor = vec3(0.5, 0.5, 0.5);
uniform float pointSize = 2.0;

void main()
{
    position = vec3(modelView * vec4(vertexPosition, 1.0));
    color = hasColors ? vertexColor.bgr : defaultColor;

    gl_Position = mvp * vec4(vertexPosition, 1.0);

    // Set point size
    gl_PointSize = pointSize;
}
