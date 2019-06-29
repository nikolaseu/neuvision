#version 150

// Splat code taken with modifications from http://www.mbroecker.com/research_pbr.html

// we accept points and render quads
layout (points) in;
layout (triangle_strip, max_vertices=4) out;

uniform mat4 modelMatrix = mat4(1.0);
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
uniform mat3 modelViewNormal;

uniform vec3 lightPosition;
uniform vec3 lightIntensity;

uniform vec3 ka;            // Ambient reflectivity
uniform vec3 kd;            // Diffuse reflectivity
uniform vec3 ks;            // Specular reflectivity
uniform float shininess;    // Specular shininess factor

// This uniform allows to manually change the splat size
uniform float splatSize = 1.0;

in VertexData
{
    vec3 position;
    vec3  color;
    vec3  normal;
    float radius;
} VertexIn[];

out FragmentData
{
    vec2 texcoord;
    vec3 color;
} VertexOut;

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
    // discard outliers
    if (VertexIn[0].radius > 0.0) {
        // Matrix setup
        mat4 mvp = projectionMatrix * viewMatrix * modelMatrix;
        mat3 normalMatrix = inverse(transpose(mat3(modelMatrix[0].xyz, modelMatrix[1].xyz, modelMatrix[2].xyz)));

        vec4 pointCenter = gl_in[0].gl_Position;
        vec3 pointNormal = normalize(normalMatrix * VertexIn[0].normal);

        // create tangent space. Helper vectors dependent on major orientation of normal
        vec3 u, v;
        if (abs(pointNormal.y) > abs(pointNormal.x)) {
            v = cross(pointNormal, vec3(1.0, 0.0, 0.0));
            u = cross(pointNormal, v);
        }
        else {
            v = cross(vec3(0.0, 1.0, 0.0), pointNormal);
            u = cross(pointNormal, v);
        }

        // Scale the splat
        u *= VertexIn[0].radius;
        v *= VertexIn[0].radius;

        // Manually alter size
        u *= (splatSize / 2.0);
        v *= (splatSize / 2.0);

        // Compute color using a simple light model
        vec3 normal = normalize(modelViewNormal * -VertexIn[0].normal);
        vec3 ambient = lightIntensity * ka;
        VertexOut.color = VertexIn[0].color * (ambient + dsModel(VertexIn[0].position, normal));

        // Calculate the four corner vertices of the quad
        vec4 a = pointCenter + vec4(-u-v, 0.0);
        vec4 b = pointCenter + vec4(-u+v, 0.0);
        vec4 c = pointCenter + vec4(+u+v, 0.0);
        vec4 d = pointCenter + vec4(+u-v, 0.0);

        // transform the four points. Note the order of output
        gl_Position = mvp * b;
        VertexOut.texcoord = vec2(-1.0, 1.0);
        EmitVertex();

        gl_Position = mvp * a;
        VertexOut.texcoord = vec2(-1.0, -1.0);
        EmitVertex();

        gl_Position = mvp * c;
        VertexOut.texcoord = vec2(1.0, 1.0);
        EmitVertex();

        gl_Position = mvp * d;
        VertexOut.texcoord = vec2(1.0, -1.0);
        EmitVertex();
    }
}
