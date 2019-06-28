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
    vertexOut.color = vertexColor;
    vertexOut.normal = vertexNormal.xyz;
    vertexOut.radius = 0.1;//vertexNormal.w; // if we want to send normal.w for radius of splat

    // if we don't have normals, force 0,0,1
    if (dot(vertexOut.normal, vertexOut.normal) < 0.001) {
        vertexOut.normal = vec3(0,0,1);
    }
}
