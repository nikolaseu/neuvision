#version 150 core

uniform mat4 viewMatrix;

uniform vec3 lightPosition;
uniform vec3 lightIntensity;

uniform vec3 ka;            // Ambient reflectivity
uniform vec3 kd;            // Diffuse reflectivity
uniform vec3 ks;            // Specular reflectivity
uniform float shininess;    // Specular shininess factor

in vec3 position;
in vec3 normal;
in vec3 color;

out vec4 fragColor;

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

    vec3 result = color * (ambient + dsModel(position, normalize(normal)));

    fragColor = vec4(result, 1.0);
}
