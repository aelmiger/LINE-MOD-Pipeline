#version 330 core
out vec4 f_color;

float near = 100.0f; 
float far  = 10000.0f; 


float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));	
}

void main()
{             
    float depth = LinearizeDepth(gl_FragCoord.z)/far / 6.5535f ; // divide by far for demonstration
    f_color =vec4(vec3(depth), 1.0);
}
