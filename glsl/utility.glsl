#version 130

int argmax(vec3 vec) {
    float crt_max = 0.0;
    int result = 0;
    float value;
    for (int i=0; i<3; ++i) {
        value = abs(vec[i]);
        if (value <= crt_max) continue;
        result = i;
        crt_max = value;
    }
    return result;
}
