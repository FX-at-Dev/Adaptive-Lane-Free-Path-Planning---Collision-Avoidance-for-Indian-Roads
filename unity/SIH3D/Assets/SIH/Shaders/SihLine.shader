// Thick world-space lines for the overlays (see LineBatch.cs).
// Unlit, vertex coloured, alpha blended; _Intensity above 1 feeds bloom.
Shader "SIH/Line"
{
    Properties
    {
        _Intensity ("Intensity", Float) = 1
        _EdgeSoftness ("Edge softness (fraction of width)", Range(0, 1)) = 0.3
        _EdgeAlpha ("Alpha at the very edge", Range(0, 1)) = 0.35
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("ZTest", Float) = 4
    }
    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }
        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            ZTest [_ZTest]
            Cull Off
            Offset -1, -1

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            float _Intensity, _EdgeSoftness, _EdgeAlpha;

            struct appdata
            {
                float4 vertex : POSITION;
                float4 dir : TEXCOORD0;   // xyz direction, w side (-1 / +1)
                float4 prm : TEXCOORD1;   // x width, y flat
                float4 color : COLOR;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                float side : TEXCOORD0;
            };

            v2f vert(appdata v)
            {
                float3 p = mul(unity_ObjectToWorld, v.vertex).xyz;
                float3 d = mul((float3x3)unity_ObjectToWorld, v.dir.xyz);
                d = d / max(length(d), 1e-6);
                float3 s;
                if (v.prm.y > 0.5)
                {
                    s = cross(float3(0, 1, 0), d);
                }
                else
                {
                    float3 view = _WorldSpaceCameraPos - p;
                    s = cross(d, view);
                }
                float ls = length(s);
                s = ls > 1e-6 ? s / ls : float3(1, 0, 0);
                p += s * (v.dir.w * v.prm.x * 0.5);

                v2f o;
                o.pos = mul(UNITY_MATRIX_VP, float4(p, 1));
                o.color = v.color;
                o.side = v.dir.w;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                // Soften the outer edge so thin lines do not shimmer.
                float e = 1 - smoothstep(1 - _EdgeSoftness, 1.0001, abs(i.side));
                float4 c = i.color;
                c.rgb *= _Intensity;
                c.a *= lerp(_EdgeAlpha, 1, e);
                return c;
            }
            ENDCG
        }
    }
}
