// A spinning LiDAR on the ego's roof, cast against the scene.
//
// In V1 this is display only: it shows what a 32-beam sensor at the
// configured mount would see of this exact world at this exact moment. It is
// not fed to the stack (V3 does that). The scene generator copies its spec
// from the run's meta, which comes from sih_config.m; every value is a field
// here and can be changed in the inspector.
//
// Points are drawn straight from a GPU buffer for every camera, including
// the Scene view, so the cloud shows in the editor preview as well as in Play.

using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;

namespace Sih
{
    [ExecuteAlways]
    public class LidarSim : MonoBehaviour
    {
        [Header("Mount")]
        [Tooltip("The sensor rides on this object (the ego).")]
        public Transform ego;
        public float mountHeight = 1.9f;

        [Header("Sensor")]
        public int beams = 32;
        public int azimuthBins = 1024;
        public float verticalFovLow = -15f, verticalFovHigh = 15f;
        public float range = 40f;
        [Tooltip("Scans per second of replay time.")]
        public float scanRate = 10f;
        [Tooltip("What the beams can hit.")]
        public LayerMask hitLayers = ~(1 << 2);

        [Header("Noise")]
        [Range(0, 1)] public float dropout = 0.05f;
        [Tooltip("Gaussian range noise, metres (1 sigma).")]
        public float rangeNoise = 0.02f;
        public uint seed = 2463534242u;

        [Header("Colouring")]
        [Tooltip("Returns below this height are ground.")]
        public float groundHeight = 0.18f;
        [Tooltip("Returns within this distance of the corridor, above the ground, are drawn hot: what perception would keep.")]
        public float roadMargin = 0.8f;
        [Tooltip("Height over which the colour runs from low to high.")]
        public float heightGradient = 6f;
        [Tooltip("Colour key of ground returns (0 = the material's Low colour, 1 = High).")]
        [Range(0, 1)] public float groundColorKey = 0.08f;
        [Tooltip("Colour key of a return at ground level; it rises with height.")]
        [Range(0, 1)] public float heightColorKeyBase = 0.25f;
        [Tooltip("Size of the grid cells used to test 'near the corridor', metres.")]
        public float roadMaskCell = 0.5f;

        [Header("Performance")]
        [Tooltip("Rays per job batch.")]
        public int raycastBatchSize = 256;
        [Tooltip("Size of the box the points are drawn in, as a multiple of the range.")]
        public float drawBoundsScale = 2.5f;

        [Header("Rendering")]
        [Tooltip("SIH/Points material: size, intensity and the low/high/hot colours are on it.")]
        public Material pointMaterial;

        [Header("Live (read only)")]
        public int pointCount;

        public bool Visible { get; set; } = true;

        RunData run;
        RoadMask mask;
        Vector3[] localDir;
        NativeArray<RaycastCommand> cmds;
        NativeArray<RaycastHit> hits;
        GraphicsBuffer buf;
        Vector4[] pts;
        MaterialPropertyBlock props;
        int specBeams, specBins;
        float specLo, specHi;
        float lastScan = float.NaN;
        Vector3 origin;
        uint rng;

        public void Bind(RunData r)
        {
            run = r;
            mask = null;
            lastScan = float.NaN;
        }

        void OnEnable() => RenderPipelineManager.beginCameraRendering += Draw;

        void OnDisable()
        {
            RenderPipelineManager.beginCameraRendering -= Draw;
            Release();
        }

        void Release()
        {
            if (cmds.IsCreated) cmds.Dispose();
            if (hits.IsCreated) hits.Dispose();
            buf?.Release();
            buf = null;
            localDir = null;
            pointCount = 0;
        }

        void Allocate()
        {
            int b = Mathf.Max(1, beams), a = Mathf.Max(8, azimuthBins);
            if (localDir != null && b == specBeams && a == specBins && specLo == verticalFovLow && specHi == verticalFovHigh) return;
            Release();
            specBeams = b; specBins = a; specLo = verticalFovLow; specHi = verticalFovHigh;
            int n = b * a;
            localDir = new Vector3[n];
            for (int i = 0; i < b; i++)
            {
                float el = b == 1 ? 0 : Mathf.Lerp(verticalFovLow, verticalFovHigh, i / (b - 1f)) * Mathf.Deg2Rad;
                for (int j = 0; j < a; j++)
                {
                    float az = j * Mathf.PI * 2f / a;
                    localDir[i * a + j] = new Vector3(Mathf.Cos(el) * Mathf.Sin(az), Mathf.Sin(el), Mathf.Cos(el) * Mathf.Cos(az));
                }
            }
            cmds = new NativeArray<RaycastCommand>(n, Allocator.Persistent);
            hits = new NativeArray<RaycastHit>(n, Allocator.Persistent);
            pts = new Vector4[n];
            buf = new GraphicsBuffer(GraphicsBuffer.Target.Structured, n, 16);
        }

        /// Scan if one is due at replay time t; force scans regardless.
        public void Tick(float t, bool visible, bool force = false)
        {
            Visible = visible;
            if (!visible || ego == null) return;
            if (!force && !float.IsNaN(lastScan) && Mathf.Abs(t - lastScan) < 1f / Mathf.Max(scanRate, 0.1f)) return;
            lastScan = t;
            Scan();
        }

        void Scan()
        {
            Allocate();
            if (mask == null && run != null) mask = new RoadMask(run, roadMargin, roadMaskCell);
            if (rng == 0) rng = seed == 0 ? 1u : seed;
            Physics.SyncTransforms();
            origin = ego.position + Vector3.up * mountHeight;
            var pose = ego.rotation;
            var q = new QueryParameters(hitLayers, false, QueryTriggerInteraction.Ignore, false);
            int n = localDir.Length;
            for (int i = 0; i < n; i++)
                cmds[i] = new RaycastCommand(origin, pose * localDir[i], q, range);
            RaycastCommand.ScheduleBatch(cmds, hits, Mathf.Max(1, raycastBatchSize), 1).Complete();

            int count = 0;
            for (int i = 0; i < n; i++)
            {
                var h = hits[i];
                if (h.distance <= 0f) continue;
                if (Rand() < dropout) continue;
                float r = h.distance + Gauss() * rangeNoise;
                Vector3 p = origin + (h.point - origin).normalized * r;
                float key;
                if (p.y < groundHeight) key = groundColorKey;
                else if (mask != null && mask.Near(p.x, p.z)) key = 2f;   // the shader's "hot" colour
                else key = Mathf.Clamp01(heightColorKeyBase + p.y / Mathf.Max(heightGradient, 0.1f));
                pts[count++] = new Vector4(p.x, p.y, p.z, key);
            }
            buf.SetData(pts, 0, 0, count);
            pointCount = count;
        }

        void Draw(ScriptableRenderContext ctx, Camera cam)
        {
            if (!Visible || pointCount == 0 || buf == null || pointMaterial == null) return;
            if (cam.cameraType == CameraType.Preview || cam.cameraType == CameraType.Reflection) return;
            props ??= new MaterialPropertyBlock();
            props.SetBuffer("_Points", buf);
            var rp = new RenderParams(pointMaterial)
            {
                camera = cam,
                worldBounds = new Bounds(origin, Vector3.one * (range * drawBoundsScale)),
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
                matProps = props,
                layer = gameObject.layer,
            };
            Graphics.RenderPrimitives(rp, MeshTopology.Triangles, pointCount * 6);
        }

        float Rand()
        {
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            return (rng & 0xFFFFFF) / 16777216f;
        }

        float Gauss()
        {
            float u = Mathf.Max(Rand(), 1e-7f), v = Rand();
            return Mathf.Sqrt(-2f * Mathf.Log(u)) * Mathf.Cos(2f * Mathf.PI * v);
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            lastScan = float.NaN;
            rng = 0;
            mask = null;   // margin or cell size may have changed
            if (!Application.isPlaying) SihReplay.RequestEditorRefresh();
        }
#endif
    }

    /// A grid marking ground within a margin of the corridor, so the per-point
    /// test in the LiDAR is a lookup rather than a search.
    public class RoadMask
    {
        readonly bool[] mask;
        readonly float x0, z0;
        readonly int w, h;
        readonly float Cell;

        public RoadMask(RunData run, float margin, float cell = 0.5f)
        {
            Cell = Mathf.Max(cell, 0.05f);
            float minx = float.MaxValue, minz = float.MaxValue, maxx = float.MinValue, maxz = float.MinValue;
            float hwMax = 0f;
            foreach (var hw in run.roadHw) hwMax = Mathf.Max(hwMax, hw);
            for (int i = 0; i < run.roadX.Length; i++)
            {
                minx = Mathf.Min(minx, run.roadX[i]); maxx = Mathf.Max(maxx, run.roadX[i]);
                minz = Mathf.Min(minz, run.roadY[i]); maxz = Mathf.Max(maxz, run.roadY[i]);
            }
            float pad = hwMax + margin + 1f;
            x0 = minx - pad; z0 = minz - pad;
            w = Mathf.CeilToInt((maxx - minx + 2 * pad) / Cell) + 1;
            h = Mathf.CeilToInt((maxz - minz + 2 * pad) / Cell) + 1;
            mask = new bool[w * h];
            for (int i = 0; i < run.roadX.Length - 1; i++)
            {
                var a = new Vector2(run.roadX[i], run.roadY[i]);
                var b = new Vector2(run.roadX[i + 1], run.roadY[i + 1]);
                float r = run.roadHw[Mathf.Min(i, run.roadHw.Length - 1)] + margin;
                int gx0 = Mathf.FloorToInt((Mathf.Min(a.x, b.x) - r - x0) / Cell), gx1 = Mathf.CeilToInt((Mathf.Max(a.x, b.x) + r - x0) / Cell);
                int gz0 = Mathf.FloorToInt((Mathf.Min(a.y, b.y) - r - z0) / Cell), gz1 = Mathf.CeilToInt((Mathf.Max(a.y, b.y) + r - z0) / Cell);
                var ab = b - a;
                float ab2 = Mathf.Max(ab.sqrMagnitude, 1e-6f);
                for (int gz = Mathf.Max(gz0, 0); gz <= Mathf.Min(gz1, h - 1); gz++)
                    for (int gx = Mathf.Max(gx0, 0); gx <= Mathf.Min(gx1, w - 1); gx++)
                    {
                        var p = new Vector2(x0 + (gx + 0.5f) * Cell, z0 + (gz + 0.5f) * Cell);
                        float u = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab2);
                        if ((p - (a + ab * u)).sqrMagnitude <= r * r) mask[gz * w + gx] = true;
                    }
            }
        }

        public bool Near(float x, float z)
        {
            int gx = (int)((x - x0) / Cell), gz = (int)((z - z0) / Cell);
            if (gx < 0 || gz < 0 || gx >= w || gz >= h) return false;
            return mask[gz * w + gx];
        }
    }
}
