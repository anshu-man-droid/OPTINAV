Shader "Hidden/OPTINAV/CameraImageDisturbance"
{
    Properties
    {
        _BlurFactor ("Blur Factor", Range(0, 1)) = 0.0
        _BlurRadiusScale ("Blur Radius Scale", Float) = 14.0
        _SensorNoiseLevel ("Sensor Noise Level", Range(0, 1)) = 0.0
        _NoiseSeed ("Noise Seed", Float) = 101.0
        _NoiseTime ("Noise Time", Float) = 0.0
        _NoiseDynamic ("Noise Dynamic", Float) = 1.0
    }

    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" }
        LOD 100

        Pass
        {
            Name "CameraImageDisturbance"
            ZWrite Off
            ZTest Always
            Blend Off
            Cull Off

            HLSLPROGRAM
            #pragma target 3.0
            #pragma vertex Vert
            #pragma fragment FragDisturbance

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"

            CBUFFER_START(UnityPerMaterial)
                float _BlurFactor;
                float _BlurRadiusScale;
                float _SensorNoiseLevel;
                float _NoiseSeed;
                float _NoiseTime;
                float _NoiseDynamic;
            CBUFFER_END

            // 12-sample Poisson disc kernel for isotropic optical defocus blur
            static const float2 POISSON_12[12] = {
                float2(-0.326212, -0.40581),
                float2(-0.840144, -0.07358),
                float2(-0.695914,  0.457137),
                float2(-0.203345,  0.620716),
                float2( 0.96234,  -0.194983),
                float2( 0.473434, -0.480026),
                float2( 0.519456,  0.767022),
                float2( 0.185461, -0.893124),
                float2( 0.507431,  0.064425),
                float2( 0.89642,   0.412458),
                float2(-0.32194,  -0.932615),
                float2(-0.791559, -0.59771)
            };

            // Fast high-quality pseudo-random hash
            float Hash21(float2 p, float seed)
            {
                float3 p3 = frac(float3(p.xyx) * float3(0.1031, 0.1030, 0.0973) + seed);
                p3 += dot(p3, p3.yzx + 33.33);
                return frac((p3.x + p3.y) * p3.z);
            }

            // Triangular pseudo-random noise in [-1, 1]
            float TriNoise(float2 pixelCoord, float seed)
            {
                float n1 = Hash21(pixelCoord, seed);
                float n2 = Hash21(pixelCoord + float2(13.1, 7.7), seed + 5.17);
                return (n1 + n2) - 1.0;
            }

            half4 FragDisturbance(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

                float2 uv = input.texcoord;
                half4 col;

                // 1. OPTICAL CAMERA BLUR
                if (_BlurFactor > 0.001)
                {
                    float2 radius = _BlurFactor * _BlurRadiusScale * _BlitTexture_TexelSize.xy;
                    half4 sum = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, uv) * 2.0;

                    [unroll]
                    for (int i = 0; i < 12; i++)
                    {
                        float2 sampleUV = uv + POISSON_12[i] * radius;
                        sum += SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, sampleUV);
                    }
                    col = sum / 14.0;
                }
                else
                {
                    col = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_LinearClamp, uv);
                }

                // 2. SENSOR / IMAGE NOISE
                if (_SensorNoiseLevel > 0.001)
                {
                    float2 pixelCoord = uv * _BlitTexture_TexelSize.zw;
                    float timeVal = (_NoiseDynamic > 0.5) ? _NoiseTime : 0.0;
                    float effectiveSeed = _NoiseSeed * 0.01 + timeVal;

                    float noise = TriNoise(pixelCoord, effectiveSeed);

                    // Signal-dependent shot noise modeling (Poisson approximation) + read noise floor
                    float lum = dot(col.rgb, half3(0.2126, 0.7152, 0.0722));
                    float shotFactor = sqrt(max(0.0, lum));
                    float noiseAmplitude = _SensorNoiseLevel * (0.5 + 0.5 * shotFactor);

                    col.rgb = max(half3(0.0, 0.0, 0.0), col.rgb + noise * noiseAmplitude);
                }

                return col;
            }
            ENDHLSL
        }
    }
    Fallback Off
}
