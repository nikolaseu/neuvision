#version 330 core

in vec3 color;

out vec4 fragColor;

void main()
{
    vec2 coord = gl_PointCoord - vec2(0.5);  //from [0,1] to [-0.5,0.5]
    if (length(coord) > 0.5) {               //outside of circle radius?
        discard;
    }

    fragColor = vec4(color, 1.0);
}
