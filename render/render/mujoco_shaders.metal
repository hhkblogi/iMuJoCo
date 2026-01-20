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
    float _padding2;  // Final alignment padding
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
                             constant Uniforms& uniforms [[buffer(1)]]) {
    float3 N = normalize(in.normal);
    float3 V = normalize(uniforms.cameraPosition - in.worldPosition);

    // Base color from vertex color (MuJoCo geom RGBA)
    float3 baseColor = in.color.rgb;

    // Light 1: Headlight (camera-attached, like MuJoCo default)
    float3 L1 = V;
    float NdotL1 = max(dot(N, L1), 0.0);

    // Light 2: Top-down fill light to illuminate horizontal surfaces (floors)
    // Unnormalized direction (0.2, 0.3, 1.0) points mostly upward (+Z) with slight
    // forward (+Y) and right (+X) offset; normalized below to create soft shadows
    // and avoid perfectly flat lighting on horizontal planes.
    float3 L2 = normalize(float3(0.2, 0.3, 1.0));
    float NdotL2 = max(dot(N, L2), 0.0);

    // Ambient component (slightly higher for better visibility)
    float3 ambient = baseColor * 0.15;

    // Diffuse from both lights
    float3 diffuse1 = baseColor * NdotL1 * 0.4;   // Headlight
    float3 diffuse2 = baseColor * NdotL2 * 0.35;  // Fill light

    // Specular component (Blinn-Phong) from headlight only
    float3 specularColor = float3(0.0);
    if (NdotL1 > 0.0 && uniforms.specular > 0.0) {
        float3 H = normalize(L1 + V);
        float NdotH = max(dot(N, H), 0.0);
        float phongExp = uniforms.shininess * 128.0;
        float spec = pow(NdotH, phongExp);
        // Scale by 0.5 to keep highlights balanced with ambient/diffuse and avoid
        // frequent saturation; overall specular intensity controlled via uniforms.specular
        specularColor = float3(0.5 * uniforms.specular * spec);
    }

    // Emission component (clamped to avoid excessive HDR overflow before final clamp)
    float3 emissive = clamp(baseColor * uniforms.emission, 0.0, 1.0);

    // Final color
    float3 finalColor = ambient + diffuse1 + diffuse2 + specularColor + emissive;

    // Clamp to valid range
    finalColor = clamp(finalColor, 0.0, 1.0);

    return float4(finalColor, in.color.a);
}
