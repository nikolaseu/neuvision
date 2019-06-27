#version 150 core

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

out vec3 vposition;
out vec3 vnormal;
out vec3 vcolor;

uniform mat4 modelMatrix;
uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;

uniform float pointSize;

void main()
{
    vnormal = normalize(modelViewNormal * vertexNormal);
    vposition = vec3(modelView * vec4(vertexPosition, 1.0));
    vcolor = vertexColor;

    gl_Position = mvp * vec4(vertexPosition, 1.0);

    // Set point size
    gl_PointSize = pointSize;
}
