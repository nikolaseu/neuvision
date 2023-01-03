#version 330

// Splat code taken with modifications from http://www.mbroecker.com/research_pbr.html

in vec3 vertexPosition;
in vec4 vertexNormal;
in vec3 vertexColor;
in float vertexRadii;

uniform vec3 defaultColor = vec3(0.5, 0.5, 0.5);
uniform bool hasColors = false;
uniform bool hasNormals = false;
uniform bool hasRadii = false;

out VertexData
{
    vec3 position;
    vec3  color;
    vec3  normal;
    float radius;
} vertexOut;

void main()
{
    gl_Position = vec4(vertexPosition, 1.0);

    vertexOut.position = vertexPosition;
    vertexOut.color = hasColors ? vertexColor.rgb : defaultColor;
    vertexOut.normal = hasNormals ? vertexNormal.xyz : vec3(0.0, 0.0, 1.0);
    vertexOut.radius = hasRadii ? vertexRadii : 0.02;
}
