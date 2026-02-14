// mujoco_shaders.metal
// Metal shaders for MuJoCo scene rendering

#include <metal_stdlib>
using namespace metal;

// MARK: - Shared Types

struct Uniforms {
    float4x4 modelMatrix;
    float4x4 viewMatrix;
    float4x4 projectionMatrix;
    float4x4 normalMatrix;
    float3 lightPosition;
    float _padding0;  // Explicit padding to match Swift struct (float3 aligns to 16 bytes)
    float3 cameraPosition;
    float _padding1;  // Explicit padding to match Swift struct
    float4 color;
    float emission;
    float specular;
    float shininess;
    float checkerboardScale;  // >0 = procedural checkerboard cell size, 0 = disabled
};

struct Light {
    float3 pos;          float _pad0;
    float3 dir;          float _pad1;
    float3 ambient;      float _pad2;
    float3 diffuse;      float _pad3;
    float3 specular;     float _pad4;
    float3 attenuation;  float cutoff;
    float exponent;
    int headlight;
    int directional;
    float _pad5;
};

struct LightBuffer {
    int lightCount;
    int _pad[3];  // align to 16 bytes
    Light lights[8];
};

struct VertexIn {
    float3 position [[attribute(0)]];
    float3 normal [[attribute(1)]];
    float2 texCoord [[attribute(2)]];
    float4 color [[attribute(3)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPosition;
    float3 normal;
    float2 texCoord;
    float4 color;
};

// MARK: - Vertex Shader

vertex VertexOut vertexMain(VertexIn in [[stage_in]],
                            constant Uniforms& uniforms [[buffer(1)]]) {
    VertexOut out;
    float4 worldPos = uniforms.modelMatrix * float4(in.position, 1.0);
    out.worldPosition = worldPos.xyz;
    out.position = uniforms.projectionMatrix * uniforms.viewMatrix * worldPos;
    out.normal = (uniforms.normalMatrix * float4(in.normal, 0.0)).xyz;
    out.texCoord = in.texCoord;
    out.color = in.color;
    return out;
}

// MARK: - Fragment Shader

fragment float4 fragmentMain(VertexOut in [[stage_in]],
                             constant Uniforms& uniforms [[buffer(1)]],
                             constant LightBuffer& lightBuf [[buffer(2)]]) {
    float3 N = normalize(in.normal);
    float3 V = normalize(uniforms.cameraPosition - in.worldPosition);
    float3 baseColor = in.color.rgb * uniforms.color.rgb;
    float alpha = in.color.a * uniforms.color.a;

    // Procedural checkerboard for ground planes
    if (uniforms.checkerboardScale > 0.0) {
        float2 cell = floor(in.worldPosition.xy / uniforms.checkerboardScale);
        bool isDark = fmod(cell.x + cell.y, 2.0) != 0.0;
        baseColor *= isDark ? 0.7 : 1.1;
    }

    float3 result = float3(0.0);

    // No lights: fall back to unlit base color + emission
    if (lightBuf.lightCount <= 0) {
        float3 unlit = baseColor + baseColor * uniforms.emission;
        return float4(clamp(unlit, 0.0, 1.0), alpha);
    }

    int numLights = min(lightBuf.lightCount, 8);
    for (int i = 0; i < numLights; i++) {
        Light light = lightBuf.lights[i];

        // Compute light direction
        float3 L;
        if (light.headlight) {
            L = V;
        } else if (light.directional) {
            L = normalize(light.dir);
        } else {
            L = normalize(light.pos - in.worldPosition);
        }

        float NdotL = max(dot(N, L), 0.0);

        // Ambient: light.ambient * baseColor (GL_COLOR_MATERIAL)
        result += light.ambient * baseColor;

        // Diffuse: light.diffuse * baseColor * NdotL
        result += light.diffuse * baseColor * NdotL;

        // Specular: Blinn-Phong
        if (NdotL > 0.0 && uniforms.specular > 0.0) {
            float3 H = normalize(L + V);
            float NdotH = max(dot(N, H), 0.0);
            float spec = pow(NdotH, uniforms.shininess * 128.0);
            result += light.specular * uniforms.specular * spec;
        }
    }

    // Emission
    result += baseColor * uniforms.emission;

    return float4(clamp(result, 0.0, 1.0), alpha);
}
