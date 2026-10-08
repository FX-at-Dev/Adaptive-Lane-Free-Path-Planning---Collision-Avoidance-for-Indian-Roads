// V3: the ego's sensors, cast against this scene, feeding the stack.
//
// Every 50 ms step the Live Link asks for one capture. The rig poses the
// world as the stack has it at that instant -- the ego at its plant state,
// every road user where Octave put it (spares stand in for judge drops) --
// senses it, and puts everything back where the picture shows it:
//
//   LiDAR   32 x 1024 rays from the roof; each return as [x y z] in the
//           sensor frame (x forward, y left, z up), with range noise and
//           dropout. Everything with a collider reflects: road users, the
//           ground, walls, trees, houses. Telling them apart is the stack's
//           job (core/perception/sih_lidar_detect.m).
//   Radar   a fan of rays over its field of view; the hits on one object
//           become one return [range bearing range-rate] (the radar's own
//           clustering), the range rate from the true relative velocity.
//   Camera  an image detector: every road user whose box is at least
//           partly unoccluded (sample rays from the lens) is reported as a
//           2D box [u0 v0 u1 v1 class score] with pixel noise, a range
//           dependent miss rate and the stack's class confusions. The view
//           it saw is rendered into a texture for the HUD's camera panel.
//
// Specs come from the stack's config at start (Use Stack Specs), or are
// set here. Nothing is created in the scene; the sensor camera is a child of
// the ego, disabled, rendered only when the rig asks.

using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

namespace Sih
{
    public class SihSensorRig : MonoBehaviour
    {
        public enum Source { Octave, Unity, Off }

        [System.Serializable]
        public struct Box
        {
            public Rect px;         // pixels, top-left origin
            public int cls;         // index into classes (0-based)
            public float score;
            public int agentId;
        }

        [Header("Where each sensor comes from (sent to the stack at start)")]
        [Tooltip("Octave: the stack's object-level model (sih_sense). Unity: this rig's raw data. Off: no such sensor.")]
        public Source cameraSource = Source.Unity;
        public Source radarSource = Source.Unity;
        public Source lidarSource = Source.Unity;
        [Tooltip("Take the sensor specs below from the stack's config when a run starts.")]
        public bool useStackSpecs = true;

        [Header("Scene")]
        public Transform ego;
        public SihReplay replay;
        [Tooltip("What the sensors can see.")]
        public LayerMask hitLayers = ~((1 << 2) | (1 << 5));
        [Tooltip("Metres from the rear axle to the centre, where the sensors sit.")]
        public float rearToCentre = 1.35f;

        [Header("LiDAR")]
        public int beams = 32;
        public int azimuthBins = 1024;
        public float lidarVFovLow = -15f, lidarVFovHigh = 15f;
        public float lidarRange = 40f;
        public float lidarMount = 1.9f;
        [Range(0, 1)] public float lidarDropout = 0.05f;
        [Tooltip("1-sigma range noise, metres.")]
        public float lidarRangeNoise = 0.02f;

        [Header("Radar")]
        public float radarRange = 90f;
        [Tooltip("Degrees.")] public float radarFov = 60f;
        [Tooltip("Degrees between beams.")] public float radarAzRes = 1f;
        [Tooltip("Degrees; one row of beams per degree step between them.")] public float radarVFovLow = -3f, radarVFovHigh = 3f;
        [Tooltip("Rows of beams across the vertical field of view.")] public int radarRows = 3;
        public float radarMount = 0.6f;
        public float radarSigmaRange = 0.25f;
        [Tooltip("Degrees.")] public float radarSigmaBearing = 2f;
        public float radarSigmaRate = 0.1f;
        [Range(0, 1)] public float radarPd = 0.9f;

        [Header("Camera")]
        public Camera sensorCamera;
        public int imageWidth = 1280, imageHeight = 720;
        [Tooltip("Horizontal field of view, degrees.")] public float cameraFov = 100f;
        public float cameraMount = 1.4f;
        public float cameraRange = 60f;
        [Tooltip("Box-edge noise, pixels.")] public float cameraSigmaPx = 1.5f;
        [Range(0, 1)] public float cameraPd = 0.92f;
        [Range(0, 1)] public float classAccuracy = 0.88f;
        public float minBoxPx = 8f;
        [Range(0, 1)] public float minVisible = 0.2f;
        [Tooltip("Render what the camera saw into its texture, for the HUD panel.")]
        public bool renderView = true;
        public RenderTexture view;
        [Tooltip("Class names in the order the stack codes them.")]
        public string[] classes = { "car", "bus", "truck", "auto", "two_wheeler", "bicycle", "pedestrian", "cattle", "pushcart", "static" };

        [Header("Static map (sent to the stack at start)")]
        [Tooltip("Colliders under these are static structure: walls, houses, trees, poles. The stack gets their footprints as its map and treats returns on them as background. Never put road users here.")]
        public Transform[] staticStructure = new Transform[0];
        public bool sendStaticMap = true;
        [Tooltip("Pieces lower than this, metres, are ground and are left out of the map.")]
        public float minStructureHeight = 0.3f;

        [Header("Noise")]
        [Tooltip("On: the sensor noise follows the world seed, so it differs with every new world too. Off: Seed below.")]
        public bool noiseFromWorld = true;
        public int seed = 26037;

        [Header("Live (read only)")]
        public int lidarPoints;
        public int radarReturns;
        public int cameraBoxes;
        public float captureMs;
        [Tooltip("Simulated time of the last capture, seconds.")]
        public float captureTime;
        [Tooltip("Real time of the last capture (Time.unscaledTime).")]
        public float captureWallTime = -1f;
        [Tooltip("The boxes the camera reported at the last capture.")]
        public List<Box> boxes = new List<Box>();

        public bool AnyUnity => cameraSource == Source.Unity || radarSource == Source.Unity || lidarSource == Source.Unity;

        System.Random rng;
        Vector3[] lidarDir;
        int dirBeams, dirBins;
        float dirLo, dirHi;

        int NoiseSeed()
        {
            if (!noiseFromWorld) return seed;
            var link = FindFirstObjectByType<SihLiveLink>();
            return link != null && link.useLive ? link.WorldSeed ^ 0x5157 : seed;
        }

        // ---- specs from the stack ---------------------------------------------

        public void ApplySpec(object spec, float rearAxleToCentre)
        {
            rearToCentre = rearAxleToCentre;
            rng = new System.Random(NoiseSeed());
            if (!useStackSpecs || spec == null) return;
            var L = Json.Get(spec, "lidar");
            lidarRange = (float)Json.D(L, "range", lidarRange);
            beams = (int)Json.D(L, "beams", beams);
            azimuthBins = (int)Json.D(L, "azimuth_bins", azimuthBins);
            var vf = Json.F(L, "v_fov");
            if (vf.Length == 2) { lidarVFovLow = vf[0] * Mathf.Rad2Deg; lidarVFovHigh = vf[1] * Mathf.Rad2Deg; }
            lidarMount = (float)Json.D(L, "mount_height", lidarMount);
            lidarDropout = (float)Json.D(L, "dropout", lidarDropout);
            lidarRangeNoise = (float)Json.D(L, "range_noise", lidarRangeNoise);

            var R = Json.Get(spec, "radar");
            radarRange = (float)Json.D(R, "range", radarRange);
            radarFov = (float)Json.D(R, "fov", radarFov * Mathf.Deg2Rad) * Mathf.Rad2Deg;
            radarAzRes = (float)Json.D(R, "az_res", radarAzRes * Mathf.Deg2Rad) * Mathf.Rad2Deg;
            var rv = Json.F(R, "v_fov");
            if (rv.Length == 2) { radarVFovLow = rv[0] * Mathf.Rad2Deg; radarVFovHigh = rv[1] * Mathf.Rad2Deg; }
            radarMount = (float)Json.D(R, "mount_height", radarMount);
            radarSigmaRange = (float)Json.D(R, "sigma_range", radarSigmaRange);
            radarSigmaBearing = (float)Json.D(R, "sigma_bear", radarSigmaBearing * Mathf.Deg2Rad) * Mathf.Rad2Deg;
            radarSigmaRate = (float)Json.D(R, "sigma_rate", radarSigmaRate);
            radarPd = (float)Json.D(R, "pd", radarPd);

            var C = Json.Get(spec, "camera");
            cameraRange = (float)Json.D(C, "range", cameraRange);
            cameraFov = (float)Json.D(C, "fov", cameraFov * Mathf.Deg2Rad) * Mathf.Rad2Deg;
            var sz = Json.F(C, "image_size");
            if (sz.Length == 2) { imageWidth = (int)sz[0]; imageHeight = (int)sz[1]; }
            cameraMount = (float)Json.D(C, "mount_height", cameraMount);
            cameraSigmaPx = (float)Json.D(C, "sigma_px", cameraSigmaPx);
            cameraPd = (float)Json.D(C, "pd", cameraPd);
            classAccuracy = (float)Json.D(C, "class_acc", classAccuracy);
            minBoxPx = (float)Json.D(C, "min_box_px", minBoxPx);
            minVisible = (float)Json.D(C, "min_visible", minVisible);
            var cl = Json.Strs(Json.Get(spec, "classes"));
            if (cl.Length > 0) classes = cl;
        }

        /// The footprints of the static structure, world frame: per piece
        /// [cx cy length width heading height], flattened.
        public List<object> StaticMap()
        {
            var flat = new List<object>();
            if (!sendStaticMap || staticStructure == null) return flat;
            foreach (var root in staticStructure)
            {
                if (root == null) continue;
                foreach (var c in root.GetComponentsInChildren<Collider>(false))
                {
                    if (!c.enabled) continue;
                    var t = c.transform;
                    float len, wid, psi, top;
                    Vector3 mid;
                    if (c is BoxCollider b)
                    {
                        var s = Vector3.Scale(b.size, t.lossyScale);
                        mid = t.TransformPoint(b.center);
                        var fw = t.forward;
                        psi = Mathf.Atan2(fw.z, fw.x);
                        len = Mathf.Abs(s.z); wid = Mathf.Abs(s.x);
                        top = mid.y + Mathf.Abs(s.y) / 2;
                    }
                    else
                    {
                        var bb = c.bounds;
                        mid = bb.center; psi = 0f;
                        len = bb.size.x; wid = bb.size.z; top = bb.max.y;
                    }
                    // Flat things (fields, paving) are ground: an animal standing
                    // on one must not vanish into the map.
                    if (top < minStructureHeight) continue;
                    flat.Add((double)mid.x); flat.Add((double)mid.z); flat.Add((double)len);
                    flat.Add((double)wid); flat.Add((double)psi); flat.Add((double)top);
                }
            }
            return flat;
        }

        // ---- one capture ------------------------------------------------------

        struct Saved { public Transform t; public Vector3 p; public Quaternion r; public bool active; }
        readonly List<Saved> saved = new List<Saved>();
        readonly Dictionary<Collider, SihAgent> ownerOf = new Dictionary<Collider, SihAgent>();
        readonly Dictionary<int, Vector2> velOf = new Dictionary<int, Vector2>();

        /// Pose the world at the stack's instant (ego state x y psi v delta a;
        /// senseAgents from the stack), sense, restore. Returns the float32
        /// payload: LiDAR rows, then radar rows, then camera rows.
        public byte[] Capture(double[] egoState, object senseAgents, out int nLidar, out int nRadar, out int nCamera)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            rng ??= new System.Random(NoiseSeed());
            nLidar = nRadar = nCamera = 0;
            var data = new List<float>(32768 * 3);

            float ex = (float)egoState[0], ey = (float)egoState[1], epsi = (float)egoState[2], ev = (float)egoState[3];
            Pose(ex, ey, epsi, senseAgents);
            try
            {
                Physics.SyncTransforms();
                var centre = Frames.EgoCentre(ex, ey, epsi, rearToCentre);
                var rot = Frames.Rot(epsi);
                var egoVel = new Vector2(Mathf.Cos(epsi), Mathf.Sin(epsi)) * ev;
                if (lidarSource == Source.Unity) nLidar = SenseLidar(centre, rot, data);
                if (radarSource == Source.Unity) nRadar = SenseRadar(centre, rot, egoVel, data);
                boxes.Clear();
                if (cameraSource == Source.Unity) nCamera = SenseCamera(centre, rot, data);
            }
            finally
            {
                Restore();
                Physics.SyncTransforms();
            }
            lidarPoints = nLidar; radarReturns = nRadar; cameraBoxes = nCamera;
            var bytes = new byte[data.Count * 4];
            System.Buffer.BlockCopy(data.ToArray(), 0, bytes, 0, bytes.Length);
            captureMs = (float)sw.Elapsed.TotalMilliseconds;
            captureWallTime = Time.unscaledTime;
            return bytes;
        }

        void Pose(float ex, float ey, float epsi, object senseAgents)
        {
            saved.Clear(); ownerOf.Clear(); velOf.Clear();
            Save(ego);
            ego.SetPositionAndRotation(Frames.EgoCentre(ex, ey, epsi, rearToCentre), Frames.Rot(epsi));

            // Everyone out of the way first; those the stack has are put back.
            var all = new List<SihAgent>();
            if (replay != null)
                foreach (var a in replay.AllAgents) if (a != null) all.Add(a);
            foreach (var a in all) Save(a.transform);
            foreach (var a in all) a.transform.position = new Vector3(0f, -1000f, 0f);

            var pose = Json.F(senseAgents, "pose");
            var cls = Json.Strs(Json.Get(senseAgents, "class"));
            for (int i = 0; i + 4 < pose.Length; i += 5)
            {
                int id = Mathf.RoundToInt(pose[i]);
                var a = replay != null ? replay.AgentFor(id, i / 5 < cls.Length ? cls[i / 5] : "") : null;
                if (a == null) continue;
                if (!a.gameObject.activeSelf) a.gameObject.SetActive(true);
                a.transform.SetPositionAndRotation(Frames.Pos(pose[i + 1], pose[i + 2]), Frames.Rot(pose[i + 3]));
                foreach (var c in a.GetComponentsInChildren<Collider>()) ownerOf[c] = a;
                velOf[id] = new Vector2(Mathf.Cos(pose[i + 3]), Mathf.Sin(pose[i + 3])) * pose[i + 4];
            }
        }

        void Save(Transform t)
        {
            if (t == null) return;
            saved.Add(new Saved { t = t, p = t.position, r = t.rotation, active = t.gameObject.activeSelf });
        }

        void Restore()
        {
            for (int i = saved.Count - 1; i >= 0; i--)
            {
                var s = saved[i];
                if (s.t == null) continue;
                s.t.SetPositionAndRotation(s.p, s.r);
                if (s.t.gameObject.activeSelf != s.active) s.t.gameObject.SetActive(s.active);
            }
            saved.Clear();
        }

        // ---- LiDAR ------------------------------------------------------------

        int SenseLidar(Vector3 centre, Quaternion rot, List<float> outp)
        {
            int b = Mathf.Max(1, beams), az = Mathf.Max(8, azimuthBins);
            if (lidarDir == null || b != dirBeams || az != dirBins || dirLo != lidarVFovLow || dirHi != lidarVFovHigh)
            {
                dirBeams = b; dirBins = az; dirLo = lidarVFovLow; dirHi = lidarVFovHigh;
                lidarDir = new Vector3[b * az];
                for (int i = 0; i < b; i++)
                {
                    float el = (b == 1 ? 0 : Mathf.Lerp(lidarVFovLow, lidarVFovHigh, i / (b - 1f))) * Mathf.Deg2Rad;
                    for (int j = 0; j < az; j++)
                    {
                        float a = j * Mathf.PI * 2f / az;
                        lidarDir[i * az + j] = new Vector3(Mathf.Cos(el) * Mathf.Sin(a), Mathf.Sin(el), Mathf.Cos(el) * Mathf.Cos(a));
                    }
                }
            }
            int n = lidarDir.Length;
            var origin = centre + Vector3.up * lidarMount;
            var q = new QueryParameters(hitLayers, false, QueryTriggerInteraction.Ignore, false);
            var cmds = new NativeArray<RaycastCommand>(n, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(n, Allocator.TempJob);
            try
            {
                for (int i = 0; i < n; i++) cmds[i] = new RaycastCommand(origin, rot * lidarDir[i], q, lidarRange);
                RaycastCommand.ScheduleBatch(cmds, hits, 256, 1).Complete();
                int count = 0;
                for (int i = 0; i < n; i++)
                {
                    var h = hits[i];
                    if (h.distance <= 0f) continue;
                    if (Rand() < lidarDropout) continue;
                    float r = h.distance + Gauss() * lidarRangeNoise;
                    var d = lidarDir[i] * r;           // sensor frame: Unity local (x right, y up, z forward)
                    outp.Add(d.z); outp.Add(-d.x); outp.Add(d.y);
                    count++;
                }
                return count;
            }
            finally { cmds.Dispose(); hits.Dispose(); }
        }

        // ---- radar ------------------------------------------------------------

        struct Group { public float rMin, bSum; public int n; public Vector2 vel; }

        int SenseRadar(Vector3 centre, Quaternion rot, Vector2 egoVel, List<float> outp)
        {
            var origin = centre + Vector3.up * radarMount;
            var groups = new Dictionary<Object, Group>();   // one per road user, or per other collider
            int rows = Mathf.Max(1, radarRows);
            float step = Mathf.Max(0.05f, radarAzRes);
            for (int ri = 0; ri < rows; ri++)
            {
                float el = (rows == 1 ? 0 : Mathf.Lerp(radarVFovLow, radarVFovHigh, ri / (rows - 1f))) * Mathf.Deg2Rad;
                for (float bd = -radarFov / 2; bd <= radarFov / 2 + 1e-3f; bd += step)
                {
                    float b = bd * Mathf.Deg2Rad;              // counter-clockwise (left) positive
                    var local = new Vector3(-Mathf.Sin(b) * Mathf.Cos(el), Mathf.Sin(el), Mathf.Cos(b) * Mathf.Cos(el));
                    if (!Physics.Raycast(origin, rot * local, out var h, radarRange, hitLayers, QueryTriggerInteraction.Ignore)) continue;
                    if (h.point.y < 0.05f) continue;             // the road itself does not return
                    Object key; Vector2 vel = Vector2.zero;
                    if (ownerOf.TryGetValue(h.collider, out var ag)) { key = ag; velOf.TryGetValue(ag.agentId, out vel); }
                    else key = h.collider;
                    groups.TryGetValue(key, out var g);
                    if (g.n == 0) g.rMin = float.MaxValue;
                    g.rMin = Mathf.Min(g.rMin, h.distance);
                    g.bSum += b; g.n++; g.vel = vel;
                    groups[key] = g;
                }
            }
            int count = 0;
            foreach (var kv in groups)
            {
                var g = kv.Value;
                float r = g.rMin, b = g.bSum / g.n;
                float pd = radarPd * (1f - 0.4f * (r / radarRange) * (r / radarRange));
                if (Rand() > pd) continue;
                float psi = Mathf.Atan2((rot * Vector3.forward).z, (rot * Vector3.forward).x) + b;   // world bearing
                var los = new Vector2(Mathf.Cos(psi), Mathf.Sin(psi));
                float rate = Vector2.Dot(g.vel - egoVel, los);
                outp.Add(r + Gauss() * radarSigmaRange);
                outp.Add(b + Gauss() * radarSigmaBearing * Mathf.Deg2Rad);
                outp.Add(rate + Gauss() * radarSigmaRate);
                count++;
            }
            return count;
        }

        // ---- camera -----------------------------------------------------------

        int SenseCamera(Vector3 centre, Quaternion rot, List<float> outp)
        {
            var lens = centre + Vector3.up * cameraMount;
            if (sensorCamera != null)
            {
                sensorCamera.transform.SetPositionAndRotation(lens, rot);
                sensorCamera.fieldOfView = 2f * Mathf.Atan(Mathf.Tan(cameraFov * 0.5f * Mathf.Deg2Rad) * imageHeight / imageWidth) * Mathf.Rad2Deg;
                if (renderView && view != null)
                {
                    sensorCamera.targetTexture = view;
                    sensorCamera.Render();
                }
            }
            float f = imageWidth * 0.5f / Mathf.Tan(cameraFov * 0.5f * Mathf.Deg2Rad);
            float cx = imageWidth * 0.5f, cy = imageHeight * 0.5f;
            var inv = Quaternion.Inverse(rot);
            int count = 0;
            var seen = new HashSet<SihAgent>();
            foreach (var kv in ownerOf)
            {
                var a = kv.Value;
                if (!seen.Add(a)) continue;
                var col = kv.Key as BoxCollider;
                if (col == null) continue;
                // The collider's eight corners and a few points inside its faces.
                var tr = col.transform;
                var pts = new List<Vector3>(17);
                for (int i = 0; i < 8; i++)
                {
                    var o = new Vector3((i & 1) == 0 ? -0.5f : 0.5f, (i & 2) == 0 ? -0.5f : 0.5f, (i & 4) == 0 ? -0.5f : 0.5f);
                    pts.Add(tr.TransformPoint(col.center + Vector3.Scale(o, col.size)));
                }
                float u0 = float.MaxValue, v0 = float.MaxValue, u1 = float.MinValue, v1 = float.MinValue;
                bool front = false;
                foreach (var p in pts)
                {
                    var c = inv * (p - lens);              // camera: x right, y up, z forward
                    float z = Mathf.Max(c.z, 0.3f);
                    if (c.z > 0.3f) front = true;
                    float u = cx + f * c.x / z, v = cy - f * c.y / z;
                    u0 = Mathf.Min(u0, u); u1 = Mathf.Max(u1, u); v0 = Mathf.Min(v0, v); v1 = Mathf.Max(v1, v);
                }
                if (!front) continue;
                if (u1 <= 0 || u0 >= imageWidth || v1 <= 0 || v0 >= imageHeight) continue;
                var mid = tr.TransformPoint(col.center);
                float range = Vector3.Distance(lens, mid);
                if (range > cameraRange) continue;

                // How much of it is unoccluded: rays from the lens to points on it.
                int vis = 0, tot = 0;
                var samples = new List<Vector3>(pts.Count + 1) { mid };
                foreach (var p in pts) samples.Add(Vector3.Lerp(mid, p, 0.8f));
                foreach (var p in samples)
                {
                    var d = p - lens;
                    if ((inv * d).z <= 0.3f) continue;
                    tot++;
                    if (Physics.Raycast(lens, d.normalized, out var h, d.magnitude + 0.05f, hitLayers, QueryTriggerInteraction.Ignore)
                        && ownerOf.TryGetValue(h.collider, out var who) && who == a) vis++;
                }
                float frac = tot > 0 ? (float)vis / tot : 0f;
                if (frac < minVisible) continue;
                float pd = cameraPd * (1f - 0.4f * (range / cameraRange) * (range / cameraRange));
                if (Rand() > pd) continue;

                int ci = System.Array.IndexOf(classes, a.agentClass);
                if (ci < 0) ci = 0;
                if (Rand() > classAccuracy) ci = Confuse(a.agentClass, ci);
                // Noise, then clipped to the image as a detector reports it: a
                // box cut off by the edge has its edge exactly on the border.
                u0 = Mathf.Clamp(u0 + Gauss() * cameraSigmaPx, 0, imageWidth);
                u1 = Mathf.Clamp(u1 + Gauss() * cameraSigmaPx, 0, imageWidth);
                v0 = Mathf.Clamp(v0 + Gauss() * cameraSigmaPx, 0, imageHeight);
                v1 = Mathf.Clamp(v1 + Gauss() * cameraSigmaPx, 0, imageHeight);
                if (u1 - u0 < 1f || v1 - v0 < minBoxPx) continue;
                outp.Add(u0); outp.Add(v0); outp.Add(u1); outp.Add(v1); outp.Add(ci + 1); outp.Add(frac);
                boxes.Add(new Box { px = Rect.MinMaxRect(u0, v0, u1, v1), cls = ci, score = frac, agentId = a.agentId });
                count++;
            }
            return count;
        }

        /// The stack's confusions (core/sensors/sih_sense.m): an auto-rickshaw
        /// reads as a car far more often than as a cow.
        int Confuse(string cls, int fallback)
        {
            string[] opts;
            switch (cls)
            {
                case "car": opts = new[] { "auto", "truck" }; break;
                case "auto": opts = new[] { "car", "pushcart" }; break;
                case "bus": opts = new[] { "truck" }; break;
                case "truck": opts = new[] { "bus", "car" }; break;
                case "two_wheeler": opts = new[] { "bicycle" }; break;
                case "bicycle": opts = new[] { "two_wheeler", "pedestrian" }; break;
                case "pedestrian": opts = new[] { "bicycle", "cattle" }; break;
                case "cattle": opts = new[] { "pedestrian", "pushcart" }; break;
                case "pushcart": opts = new[] { "cattle", "auto" }; break;
                default: opts = new[] { "car" }; break;
            }
            int i = System.Array.IndexOf(classes, opts[Mathf.Min(opts.Length - 1, (int)(Rand() * opts.Length))]);
            return i >= 0 ? i : fallback;
        }

        float Rand() => (float)rng.NextDouble();

        float Gauss()
        {
            double u = 1.0 - rng.NextDouble(), v = rng.NextDouble();
            return (float)(System.Math.Sqrt(-2.0 * System.Math.Log(u)) * System.Math.Cos(2.0 * System.Math.PI * v));
        }
    }
}
