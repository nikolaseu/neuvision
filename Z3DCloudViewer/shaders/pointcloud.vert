#version 150

// from Wikipedia: https://en.wikipedia.org/wiki/Blinn–Phong_shading_model

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

out vec3 vertPos;
out vec3 normalInterp;
out vec3 color;

uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;

uniform float pointSize;

void main()
{
    vec4 vertPos4 = modelView * vec4(vertexPosition, 1.0);

    vertPos = vec3(vertPos4) / vertPos4.w;
    normalInterp = normalize(modelViewNormal * vertexNormal);
    color = vertexColor;

    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;
}
