// The scenery generator: houses, garden walls, trees, poles, fields and
// road-side objects (cones, barriers, signs, milestones) placed along the
// road from the prefab catalog, by rules, new on every Play.
//
// Each rule says which side of the road, how far beyond its edge, how far
// apart along it, how likely, how big and which way to face. Nothing is put
// on the road, on another object, at the ego's start, at the goal or across
// a cross street. A seed reproduces a layout exactly; with the Live Link the
// seed is the world seed, shared with the traffic the stack generates.
//
// In edit mode: Preview layout generates one into the scene to look at and
// edit; Keep this layout turns it into ordinary saved objects and stops
// regenerating on Play; Clear removes it. The generated objects are the
// static structure the sensors' map is made of (SihSensorRig).

using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sih
{
    [DefaultExecutionOrder(-100)]
    public class SihSceneryGenerator : MonoBehaviour
    {
        public enum Side { Both, Left, Right }
        public enum Facing { FaceRoad, AlongRoad, Random }

        [Serializable]
        public class Rule
        {
            public string name = "trees";
            public bool enabled = true;
            [Tooltip("Catalog scenery key: house, tree, wall, pole, field, cone, barrier, sign, milestone.")]
            public string kind = "tree";
            public Side side = Side.Both;
            [Tooltip("Metres between placements along the road.")]
            public float spacingMin = 6f, spacingMax = 12f;
            [Range(0, 1)] public float probability = 0.6f;
            [Tooltip("Placed this many at a time, scattered.")]
            public int clusterMin = 1, clusterMax = 1;
            [Tooltip("From the road edge to the object's near side, metres.")]
            public float offsetMin = 2.5f, offsetMax = 9f;
            [Tooltip("Size: across (x), deep (z), tall (y), metres. Prefabs are scaled to it.")]
            public Vector2 width = new Vector2(2.8f, 5.2f);
            public Vector2 depth = new Vector2(2.8f, 5.2f);
            public Vector2 height = new Vector2(4.5f, 8f);
            [Tooltip("Keep width and depth equal (trees, poles).")]
            public bool squareFootprint = true;
            public Facing facing = Facing.Random;
            [Tooltip("Degrees.")] public float yawJitter = 0f;
            [Tooltip("Metres of scatter along the road.")] public float alongJitter = 1f;
            [Tooltip("Room kept around it from other objects, as a fraction of its size.")]
            public float clearance = 0.6f;
        }

        [Header("Source")]
        public SihCatalog catalog;
        [Tooltip("The run whose road to dress (the scene's Replay run file).")]
        public SihReplay replay;
        [Tooltip("Takes the world seed from here when it is driving live.")]
        public SihLiveLink liveLink;

        [Header("Layout")]
        [Tooltip("Make a new layout every time Play starts.")]
        public bool randomizeOnPlay = true;
        [Tooltip("0: a new random seed every Play (or the Live Link's world seed).")]
        public int seed;
        [Tooltip("Generated objects go under this (created if missing).")]
        public Transform generatedRoot;
        [Tooltip("Keep this far from the ego's start and the goal, metres.")]
        public float startClear = 12f, goalClear = 8f;
        [Tooltip("Never closer to the road edge than this, metres.")]
        public float roadMargin = 0.4f;
        [Tooltip("Cross streets (station, half width) are kept clear, plus this, metres.")]
        public float crossStreetMargin = 3f;
        public Rule[] rules = DefaultRules();

        [Header("Live (read only)")]
        public int lastSeed;
        public int placed;

        void Awake()
        {
            if (!Application.isPlaying || !randomizeOnPlay) return;
            // A kept layout is for editing; Play shows a new random one instead.
            var kept = transform.Find("Kept");
            if (kept != null) kept.gameObject.SetActive(false);
            Generate(ResolveSeed());
        }

        public int ResolveSeed()
        {
            // Not isActiveAndEnabled: that is false until the link's own
            // OnEnable, and this runs first, in Awake.
            if (liveLink != null && liveLink.enabled && liveLink.gameObject.activeInHierarchy && liveLink.useLive)
                return liveLink.WorldSeed;
            return seed != 0 ? seed : Environment.TickCount & 0x7fffffff;
        }

        Transform Root()
        {
            if (generatedRoot == null)
            {
                var t = transform.Find("Generated");
                if (t == null) { t = new GameObject("Generated").transform; t.SetParent(transform, false); }
                generatedRoot = t;
            }
            return generatedRoot;
        }

        public void Clear()
        {
            var root = Root();
            for (int i = root.childCount - 1; i >= 0; i--)
            {
                var c = root.GetChild(i).gameObject;
                if (Application.isPlaying) Destroy(c); else DestroyImmediate(c);
            }
            placed = 0;
        }

        // ---- the road ---------------------------------------------------------

        RunData run;
        float[] arc;

        bool BindRoad()
        {
            if (replay == null || replay.runFile == null) return false;
            run = SihReplay.ParseCached(replay.runFile);
            int n = run.roadX.Length;
            arc = new float[n];
            for (int i = 1; i < n; i++)
                arc[i] = arc[i - 1] + Vector2.Distance(new Vector2(run.roadX[i], run.roadY[i]), new Vector2(run.roadX[i - 1], run.roadY[i - 1]));
            return n > 1;
        }

        void Pose(float s, out Vector2 c, out float psi, out float hw)
        {
            int n = arc.Length, i = 0;
            while (i < n - 2 && arc[i + 1] < s) i++;
            float u = (s - arc[i]) / Mathf.Max(arc[i + 1] - arc[i], 1e-4f);
            c = Vector2.LerpUnclamped(new Vector2(run.roadX[i], run.roadY[i]), new Vector2(run.roadX[i + 1], run.roadY[i + 1]), u);
            psi = Frames.LerpAngle(run.roadPsi[i], run.roadPsi[i + 1], Mathf.Clamp01(u));
            hw = Mathf.Lerp(run.roadHw[Mathf.Min(i, run.roadHw.Length - 1)], run.roadHw[Mathf.Min(i + 1, run.roadHw.Length - 1)], Mathf.Clamp01(u));
        }

        float DistToRoad(Vector2 p)
        {
            float best = float.MaxValue;
            for (int i = 0; i < run.roadX.Length - 1; i++)
            {
                var a = new Vector2(run.roadX[i], run.roadY[i]);
                var ab = new Vector2(run.roadX[i + 1], run.roadY[i + 1]) - a;
                float u = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-6f));
                best = Mathf.Min(best, Vector2.Distance(p, a + ab * u) - run.roadHw[Mathf.Min(i, run.roadHw.Length - 1)]);
            }
            return best;
        }

        // ---- generation -------------------------------------------------------

        readonly List<Vector3> taken = new List<Vector3>();   // x, y, radius
        readonly Dictionary<GameObject, Vector3> nominal = new Dictionary<GameObject, Vector3>();

        /// A new layout from seed: everything previously generated goes.
        public void Generate(int s)
        {
            Clear();
            if (catalog == null || !BindRoad()) return;
            lastSeed = s;
            var rng = new System.Random(s);
            float R(float a, float b) => a + (float)rng.NextDouble() * (b - a);
            taken.Clear();
            float total = arc[arc.Length - 1];
            var start = new Vector2(run.roadX[0], run.roadY[0]);
            var goal = run.goal;

            foreach (var rule in rules)
            {
                if (rule == null || !rule.enabled) continue;
                for (int side = -1; side <= 1; side += 2)
                {
                    if ((rule.side == Side.Left && side < 0) || (rule.side == Side.Right && side > 0)) continue;
                    float st = R(-20f, 0f);
                    while (st < total + 30f)
                    {
                        st += R(Mathf.Max(0.5f, rule.spacingMin), Mathf.Max(rule.spacingMin, rule.spacingMax));
                        if (rng.NextDouble() > rule.probability) continue;
                        if (NearCrossStreet(st)) continue;
                        int k = rng.Next(Mathf.Max(1, rule.clusterMin), Mathf.Max(rule.clusterMin, rule.clusterMax) + 1);
                        for (int j = 0; j < k; j++)
                            PlaceOne(rule, side, st + R(-rule.alongJitter, rule.alongJitter), rng, start, goal);
                    }
                }
            }
        }

        bool NearCrossStreet(float s)
        {
            var cs = run.crossStreets;
            for (int i = 0; i + 1 < cs.Length; i += 2)
                if (Mathf.Abs(s - cs[i]) < cs[i + 1] + crossStreetMargin) return true;
            return false;
        }

        void PlaceOne(Rule rule, int side, float s, System.Random rng, Vector2 start, Vector2 goal)
        {
            float R(float a, float b) => a + (float)rng.NextDouble() * (b - a);
            var prefab = catalog.Scenery(rule.kind, rng);
            if (prefab == null) return;
            Pose(s, out var c, out float psi, out float hw);
            var nrm = new Vector2(-Mathf.Sin(psi), Mathf.Cos(psi)) * side;
            var fwd = new Vector2(Mathf.Cos(psi), Mathf.Sin(psi));

            float w = R(rule.width.x, rule.width.y);
            float d = rule.squareFootprint ? w : R(rule.depth.x, rule.depth.y);
            float h = R(rule.height.x, rule.height.y);

            Quaternion rot;
            float across;   // its extent away from the road
            switch (rule.facing)
            {
                case Facing.FaceRoad:
                    rot = Quaternion.LookRotation(-new Vector3(nrm.x, 0, nrm.y)); across = d; break;
                case Facing.AlongRoad:
                    rot = Frames.Rot(psi); across = w; break;
                default:
                    rot = Quaternion.Euler(0, R(0f, 360f), 0); across = Mathf.Max(w, d); break;
            }
            rot = Quaternion.Euler(0, R(-rule.yawJitter, rule.yawJitter), 0) * rot;

            var p = c + nrm * (hw + R(rule.offsetMin, rule.offsetMax) + across / 2);
            float radius = Mathf.Max(w, d) * 0.5f * (1f + rule.clearance);
            if (Vector2.Distance(p, start) < startClear + radius || Vector2.Distance(p, goal) < goalClear + radius) return;
            if (DistToRoad(p) < across / 2 + roadMargin) return;
            foreach (var q in taken)
                if ((new Vector2(q.x, q.y) - p).sqrMagnitude < (radius + q.z) * (radius + q.z)) return;

            var go = Instantiate(prefab, Root());
            go.name = $"{rule.name} {placed + 1}";
            go.transform.SetPositionAndRotation(Frames.Pos(p.x, p.y), rot);
            var size = Nominal(prefab);
            go.transform.localScale = Vector3.Scale(prefab.transform.localScale,
                new Vector3(w / Mathf.Max(size.x, 0.01f), h / Mathf.Max(size.y, 0.01f), d / Mathf.Max(size.z, 0.01f)));
            taken.Add(new Vector3(p.x, p.y, radius));
            placed++;
        }

        /// The prefab's size as built (its colliders, else its renderers).
        Vector3 Nominal(GameObject prefab)
        {
            if (nominal.TryGetValue(prefab, out var v)) return v;
            var b = new Bounds(Vector3.zero, Vector3.zero);
            bool any = false;
            foreach (var col in prefab.GetComponentsInChildren<BoxCollider>(true))
            {
                var t = col.transform;
                var cb = new Bounds(t.localPosition + Vector3.Scale(col.center, t.localScale), Vector3.Scale(col.size, t.localScale));
                if (!any) { b = cb; any = true; } else b.Encapsulate(cb);
            }
            if (!any)
                foreach (var r in prefab.GetComponentsInChildren<MeshFilter>(true))
                {
                    if (r.sharedMesh == null) continue;
                    var t = r.transform;
                    var mb = r.sharedMesh.bounds;
                    var cb = new Bounds(t.localPosition + Vector3.Scale(mb.center, t.localScale), Vector3.Scale(mb.size, t.localScale));
                    if (!any) { b = cb; any = true; } else b.Encapsulate(cb);
                }
            v = any ? b.size : Vector3.one;
            if (v.y < 0.05f) v.y = 1f;                 // flat things keep their height
            nominal[prefab] = v;
            return v;
        }

        /// Turn the generated objects into ordinary scene objects that stay.
        public void Keep()
        {
            var root = Root();
            var kept = transform.Find("Kept");
            if (kept == null) { kept = new GameObject("Kept").transform; kept.SetParent(transform, false); }
            for (int i = root.childCount - 1; i >= 0; i--) root.GetChild(i).SetParent(kept, true);
            randomizeOnPlay = false;
        }

        public static Rule[] DefaultRules() => new[]
        {
            new Rule { name = "House", kind = "house", spacingMin = 9f, spacingMax = 18f, probability = 0.55f, offsetMin = 3.5f, offsetMax = 6f,
                       width = new Vector2(5f, 8f), depth = new Vector2(4.5f, 7f), height = new Vector2(4.0f, 5.4f), squareFootprint = false,
                       facing = Facing.FaceRoad, yawJitter = 4f, clearance = 0.25f },
            new Rule { name = "Garden wall", kind = "wall", spacingMin = 12f, spacingMax = 24f, probability = 0.35f, offsetMin = 1.6f, offsetMax = 2.4f,
                       width = new Vector2(0.3f, 0.4f), depth = new Vector2(4f, 7f), height = new Vector2(1.0f, 1.3f), squareFootprint = false,
                       facing = Facing.AlongRoad, clearance = 0.05f },
            new Rule { name = "Tree", kind = "tree", spacingMin = 6f, spacingMax = 13f, probability = 0.6f, clusterMin = 1, clusterMax = 3,
                       offsetMin = 2.5f, offsetMax = 9f, width = new Vector2(2.8f, 5.2f), height = new Vector2(4.5f, 8f), alongJitter = 4f, clearance = 0.2f },
            new Rule { name = "Pole", kind = "pole", side = Side.Left, spacingMin = 30f, spacingMax = 45f, probability = 0.9f, offsetMin = 1.2f, offsetMax = 1.8f,
                       width = new Vector2(0.3f, 0.3f), height = new Vector2(7f, 8f), facing = Facing.AlongRoad, clearance = 1f },
            new Rule { name = "Field", kind = "field", spacingMin = 20f, spacingMax = 40f, probability = 0.6f, offsetMin = 10f, offsetMax = 25f,
                       width = new Vector2(8f, 16f), depth = new Vector2(8f, 16f), height = new Vector2(0.05f, 0.05f), squareFootprint = false,
                       facing = Facing.AlongRoad, yawJitter = 15f, clearance = 0f },
            new Rule { name = "Milestone", kind = "milestone", side = Side.Left, spacingMin = 90f, spacingMax = 110f, probability = 0.9f,
                       offsetMin = 0.6f, offsetMax = 1.0f, width = new Vector2(0.4f, 0.4f), depth = new Vector2(0.25f, 0.25f),
                       height = new Vector2(0.7f, 0.8f), squareFootprint = false, facing = Facing.FaceRoad, clearance = 1f },
            new Rule { name = "Sign", kind = "sign", spacingMin = 50f, spacingMax = 90f, probability = 0.4f, offsetMin = 0.8f, offsetMax = 1.5f,
                       width = new Vector2(0.8f, 0.9f), depth = new Vector2(0.1f, 0.1f), height = new Vector2(2.2f, 2.6f), squareFootprint = false,
                       facing = Facing.FaceRoad, yawJitter = 20f, clearance = 1f },
            new Rule { name = "Cone", kind = "cone", spacingMin = 40f, spacingMax = 90f, probability = 0.3f, clusterMin = 2, clusterMax = 4,
                       offsetMin = 0.3f, offsetMax = 0.8f, width = new Vector2(0.4f, 0.4f), height = new Vector2(0.6f, 0.75f), alongJitter = 2f, clearance = 0.5f },
            new Rule { name = "Barrier", kind = "barrier", spacingMin = 60f, spacingMax = 120f, probability = 0.2f, offsetMin = 0.5f, offsetMax = 1.0f,
                       width = new Vector2(0.4f, 0.5f), depth = new Vector2(1.8f, 2.4f), height = new Vector2(0.9f, 1.0f), squareFootprint = false,
                       facing = Facing.AlongRoad, clearance = 0.3f },
        };
    }
}
