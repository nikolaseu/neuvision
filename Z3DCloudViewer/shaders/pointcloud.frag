#version 330

in FragmentData
{
    vec2 texcoord;
    vec3 color;
} FragmentIn;

out vec4 fragmentColor;

void main()
{
    // Splat code taken with modifications from http://www.mbroecker.com/research_pbr.html
    // calculate whether this fragment is inside or outside the splat circle
    // tex coords from -1.0 to 1.0
    if (pow(FragmentIn.texcoord.x, 2.0) + pow(FragmentIn.texcoord.y, 2.0) > 1.0) {
        discard;
    }

    // FAST
    fragmentColor = vec4(FragmentIn.color, 1.0);

    // SLOW! we compute the shaded color in the geometry shader, only once per point
//    vec3 ambient = lightIntensity * ka;
//    vec3 result = FragmentIn.color * (ambient + dsModel(FragmentIn.position, normalize(FragmentIn.normal)));
//    fragmentColor = vec4(result, 1.0);
}
