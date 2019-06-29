#version 330

// Splat code taken with modifications from http://www.mbroecker.com/research_pbr.html

// The vertex data. The normal stores the normal and the
// calculated radius of the splat.
//layout(location = 0) in vec3 vertexPosition;
//layout(location = 1) in vec3 vertexColor;
//layout(location = 2) in vec4 vertexNormal;

in vec3 vertexPosition;
in vec4 vertexNormal;
in vec3 vertexColor;

uniform bool hasColors = false;
uniform vec3 defaultColor = vec3(0.5, 0.5, 0.5);

uniform bool hasNormals = false;

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
    vertexOut.color = hasColors ? vertexColor : defaultColor;
    vertexOut.normal = hasNormals ? vertexNormal.xyz : vec3(0.0,0.0,1.0);
    vertexOut.radius = 0.1;//vertexNormal.w; // if we want to send normal.w for radius of splat
}
