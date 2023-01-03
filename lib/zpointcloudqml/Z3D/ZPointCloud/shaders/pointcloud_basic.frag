#version 450 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;

layout(location = 0) out vec4 fragColor;

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
};

layout(std140, binding = auto) uniform frag_uniforms {
    vec3 lightPosition;
    vec3 lightIntensity;

    vec3 ka;            // Ambient reflectivity
    vec3 kd;            // Diffuse reflectivity
    vec3 ks;            // Specular reflectivity
    float shininess;    // Specular shininess factor
};

vec3 dsModel(const in vec3 pos, const in vec3 n)
{
    // Calculate the vector from the light to the fragment
    vec3 s = normalize(vec3(viewMatrix * vec4(lightPosition, 1.0)) - pos);

    // Calculate the vector from the fragment to the eye position
    // (origin since this is in "eye" or "camera" space)
    vec3 v = normalize(-pos);

    // Reflect the light beam using the normal at this fragment
    vec3 r = reflect(-s, n);

    // Calculate the diffuse component
    float diffuse = max(dot(s, n), 0.0);

    // Calculate the specular component
    float specular = 0.0;
    if (dot(s, n) > 0.0) {
        specular = pow(max(dot(r, v), 0.0), shininess);
    }

    // Combine the diffuse and specular contributions (ambient is taken into account by the caller)
    return lightIntensity * (kd * diffuse + ks * specular);
}

void main()
{
    vec2 coord = gl_PointCoord - vec2(0.5);  //from [0,1] to [-0.5,0.5]
    if (length(coord) > 0.5) {               //outside of circle radius?
        discard;
    }

    vec3 ambient = lightIntensity * ka;
    fragColor = vec4(color * (ambient + dsModel(position, normal)), 1);

//    fragColor = vec4((normal + 1.0) * 0.5, 1);
}
