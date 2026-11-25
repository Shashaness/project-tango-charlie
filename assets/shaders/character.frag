#version 330 core

in vec3 vertexColor;
uniform sampler2D baseColor;
uniform vec3 baseFactor;
out vec4 fragmentColor;

void main()
{
    fragmentColor = vec4(texture(baseColor, vertexColor.xy).rgb * baseFactor, 1.0);
}
