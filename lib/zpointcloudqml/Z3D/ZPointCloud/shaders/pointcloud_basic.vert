#version 450 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec4 vertexNormal;
layout(location = 2) in vec3 vertexColor;

layout(location = 0) out vec3 position;
layout(location = 1) out vec3 normal;
layout(location = 2) out vec3 color;

layout(std140, binding = 0) uniform qt3d_render_view_uniforms {
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 uncorrectedProjectionMatrix;
    mat4 clipCorrectionMatrix;
    mat4 viewProjectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
    mat4 inverseViewProjectionMatrix;
    mat4 viewportMatrix;
    mat4 inverseViewportMatrix;
    vec4 textureTransformMatrix;
    vec3 eyePosition;
    float aspectRatio;
    float gamma;
    float exposure;
    float time;
    float yUpInNDC;
    float yUpInFBO;
};

layout(std140, binding = 1) uniform qt3d_command_uniforms {
    mat4 modelMatrix;
    mat4 inverseModelMatrix;
    mat4 modelViewMatrix;
    mat3 modelNormalMatrix;
    mat4 inverseModelViewMatrix;
    mat4 modelViewProjection;
    mat4 inverseModelViewProjectionMatrix;
    mat3 modelViewNormal;
};

layout(std140, binding = auto) uniform vert_uniforms {
    bool hasColors;
    vec3 defaultColor;
    float pointSize;
};

void main()
{
    position = vec3(modelViewMatrix * vec4(vertexPosition, 1.0));
    normal = normalize(modelViewNormal * vertexNormal.xyz);
    color = hasColors ? vertexColor.bgr : defaultColor;

    gl_Position = modelViewProjection * vec4(vertexPosition, 1.0);

    // Set point size
    gl_PointSize = pointSize * 2;
}
