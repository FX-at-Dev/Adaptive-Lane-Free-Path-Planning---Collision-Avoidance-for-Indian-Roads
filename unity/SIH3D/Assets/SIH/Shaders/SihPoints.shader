// LiDAR returns as camera-facing squares, read straight from a buffer
// (Graphics.RenderPrimitives, 6 vertices per point). w of each point is a
// colour key: 0..1 height along the gradient, 2 a return inside the
// drivable corridor above the ground, which is what perception would keep.
Shader "SIH/Points"
{
    Properties
    {
        _Size ("Size", Float) = 0.07
        _SizePerMetre ("Size growth per metre of distance", Float) = 0.004
        _Intensity ("Intensity", Float) = 1.6
        _Low ("Low", Color) = (0.05, 0.25, 0.45, 1)
        _High ("High", Color) = (0.55, 0.95, 1.0, 1)
        _Hot ("Hot", Color) = (1.0, 0.55, 0.25, 1)
        _Alpha ("Alpha (low/high)", Range(0, 1)) = 0.85
        _HotAlpha ("Alpha (hot)", Range(0, 1)) = 1
    }
    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }
        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            StructuredBuffer<float4> _Points;
            float _Size, _SizePerMetre, _Intensity, _Alpha, _HotAlpha;
            float4 _Low, _High, _Hot;

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                float2 uv : TEXCOORD0;
            };

            static const float2 corner[6] =
            {
                float2(-1, -1), float2(1, -1), float2(1, 1),
                float2(-1, -1), float2(1, 1), float2(-1, 1)
            };

            v2f vert(uint vid : SV_VertexID)
            {
                float4 pt = _Points[vid / 6];
                float2 c = corner[vid % 6];
                float3 right = UNITY_MATRIX_V[0].xyz;
                float3 up = UNITY_MATRIX_V[1].xyz;
                float dist = length(_WorldSpaceCameraPos - pt.xyz);
                float size = _Size * (1 + dist * _SizePerMetre);
                float3 p = pt.xyz + (right * c.x + up * c.y) * size * 0.5;

                v2f o;
                o.pos = mul(UNITY_MATRIX_VP, float4(p, 1));
                o.uv = c;
                float4 col = pt.w >= 1.5 ? _Hot : lerp(_Low, _High, saturate(pt.w));
                col.rgb *= _Intensity;
                col.a = pt.w >= 1.5 ? _HotAlpha : _Alpha;
                o.color = col;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                float r = dot(i.uv, i.uv);
                clip(1 - r);
                return i.color;
            }
            ENDCG
        }
    }
}
