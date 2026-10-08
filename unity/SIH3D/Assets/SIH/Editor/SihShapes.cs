// The stylised low-poly models and the material library, built once in the
// editor and saved as assets: materials in Assets/SIH/Materials, meshes in
// Assets/SIH/Meshes, and (via SihSceneBuilder) one prefab per road-user class
// and per scenery piece. Edit any of those assets and every scene that uses
// them follows; nothing here runs in Play.

using System.Collections.Generic;
using System.IO;
using Sih;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace SihEditor
{
    public static class SihShapes
    {
        public const string MatDir = "Assets/SIH/Materials";
        public const string MeshDir = "Assets/SIH/Meshes";

        /// When set, existing materials and meshes are reset to the defaults
        /// below; otherwise they are left exactly as edited.
        public static bool overwrite;

        static readonly Dictionary<string, Material> mats = new Dictionary<string, Material>();

        public static void ClearCache() => mats.Clear();

        public static void EnsureDir(string assetDir)
        {
            if (AssetDatabase.IsValidFolder(assetDir)) return;
            var parent = Path.GetDirectoryName(assetDir).Replace('\\', '/');
            EnsureDir(parent);
            AssetDatabase.CreateFolder(parent, Path.GetFileName(assetDir));
        }

        static Material Asset(string name, Shader shader, System.Action<Material> init)
        {
            if (mats.TryGetValue(name, out var m) && m != null) return m;
            EnsureDir(MatDir);
            string path = $"{MatDir}/{name}.mat";
            m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                m = new Material(shader) { name = name };
                init(m);
                AssetDatabase.CreateAsset(m, path);
            }
            else if (overwrite)
            {
                m.shader = shader;
                init(m);
                EditorUtility.SetDirty(m);
            }
            mats[name] = m;
            return m;
        }

        /// URP Lit in one colour; emission above 0 glows (and blooms above 1).
        public static Material Lit(Color c, float smooth = 0.25f, float emission = 0f)
        {
            string name = $"Lit {ColorUtility.ToHtmlStringRGB(c)} s{Mathf.RoundToInt(smooth * 100):00}" +
                          (emission > 0f ? $" e{emission:0.0}" : "");
            return Asset(name, Shader.Find("Universal Render Pipeline/Lit"), m =>
            {
                m.SetColor("_BaseColor", c);
                m.SetFloat("_Smoothness", smooth);
                m.SetFloat("_Metallic", 0f);
                if (emission > 0f)
                {
                    m.EnableKeyword("_EMISSION");
                    m.SetColor("_EmissionColor", c * emission);
                    m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
                }
            });
        }

        /// Overlay lines: unlit, vertex coloured; intensity above 1 blooms.
        public static Material Line(string name, float intensity, bool depthTest, int queueOffset)
        {
            return Asset("Line " + name, Shader.Find("SIH/Line"), m =>
            {
                m.SetFloat("_Intensity", intensity);
                m.SetFloat("_ZTest", depthTest ? 4f : 8f);   // LessEqual : Always
                m.renderQueue = 3000 + queueOffset;
            });
        }

        public static Material Points() => Asset("LiDAR Points", Shader.Find("SIH/Points"), m => { });

        // ---- meshes ----------------------------------------------------------

        static Mesh MeshAsset(string name, System.Func<Mesh> make)
        {
            EnsureDir(MeshDir);
            string path = $"{MeshDir}/{name}.asset";
            var m = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            if (m != null && !overwrite) return m;
            var fresh = make();
            fresh.name = name;
            if (m == null) { AssetDatabase.CreateAsset(fresh, path); return fresh; }
            EditorUtility.CopySerialized(fresh, m);
            Object.DestroyImmediate(fresh);
            EditorUtility.SetDirty(m);
            return m;
        }

        /// Gable roof: unit footprint, apex along z, height 1.
        public static Mesh Prism() => MeshAsset("Roof Prism", () =>
        {
            var m = new Mesh();
            var v = new List<Vector3>();
            var tri = new List<int>();
            void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
            {
                int i = v.Count; v.Add(a); v.Add(b); v.Add(c); v.Add(d);
                tri.AddRange(new[] { i, i + 1, i + 2, i, i + 2, i + 3 });
            }
            void Tri(Vector3 a, Vector3 b, Vector3 c)
            {
                int i = v.Count; v.Add(a); v.Add(b); v.Add(c);
                tri.AddRange(new[] { i, i + 1, i + 2 });
            }
            float h = 0.5f;
            var l0 = new Vector3(-h, 0, -h); var l1 = new Vector3(-h, 0, h);
            var r0 = new Vector3(h, 0, -h); var r1 = new Vector3(h, 0, h);
            var a0 = new Vector3(0, 1, -h); var a1 = new Vector3(0, 1, h);
            Quad(l0, a0, a1, l1);
            Quad(r1, a1, a0, r0);
            Tri(l1, a1, r1);
            Tri(r0, a0, l0);
            m.SetVertices(v);
            m.SetTriangles(tri, 0);
            m.RecalculateNormals();
            m.RecalculateBounds();
            return m;
        });

        /// Unit disc on the ground, for markers.
        public static Mesh Disc() => MeshAsset("Disc", () =>
        {
            const int seg = 48;
            var m = new Mesh();
            var v = new Vector3[seg + 1];
            var tri = new int[seg * 3];
            for (int i = 0; i < seg; i++)
            {
                float a = i * Mathf.PI * 2 / seg;
                v[i + 1] = new Vector3(Mathf.Cos(a) * 0.5f, 0, Mathf.Sin(a) * 0.5f);
                tri[3 * i] = 0; tri[3 * i + 1] = 1 + (i + 1) % seg; tri[3 * i + 2] = 1 + i;
            }
            m.vertices = v; m.triangles = tri;
            m.RecalculateNormals(); m.RecalculateBounds();
            return m;
        });

        // ---- parts -----------------------------------------------------------

        public static GameObject Part(Transform parent, PrimitiveType t, Vector3 pos, Vector3 scale,
                                      Color c, float smooth = 0.25f, float emission = 0f, string name = null)
        {
            var g = GameObject.CreatePrimitive(t);
            Object.DestroyImmediate(g.GetComponent<Collider>());
            g.name = name ?? t.ToString().ToLowerInvariant();
            g.transform.SetParent(parent, false);
            g.transform.localPosition = pos;
            g.transform.localScale = scale;
            var r = g.GetComponent<MeshRenderer>();
            r.sharedMaterial = Lit(c, smooth, emission);
            r.shadowCastingMode = ShadowCastingMode.On;
            return g;
        }

        public static GameObject MeshPart(Transform parent, Mesh mesh, Vector3 pos, Vector3 scale, Material mat, string name)
        {
            var g = new GameObject(name);
            g.transform.SetParent(parent, false);
            g.transform.localPosition = pos;
            g.transform.localScale = scale;
            g.AddComponent<MeshFilter>().sharedMesh = mesh;
            g.AddComponent<MeshRenderer>().sharedMaterial = mat;
            return g;
        }

        static GameObject Wheel(Transform p, float x, float z, float r, float w)
        {
            var g = Part(p, PrimitiveType.Cylinder, new Vector3(x, r, z), new Vector3(2 * r, w / 2, 2 * r),
                         Palette.Hex("#1b1f27"), 0.1f, 0f, "wheel");
            g.transform.localRotation = Quaternion.Euler(0, 0, 90);
            return g;
        }

        static GameObject Root(string name, float L, float W, float H, bool collider)
        {
            var g = new GameObject(name);
            if (collider)
            {
                var bc = g.AddComponent<BoxCollider>();
                bc.size = new Vector3(W, H, L);
                bc.center = new Vector3(0, H / 2, 0);
            }
            return g;
        }

        public static float HeightOf(string cls)
        {
            switch (cls)
            {
                case "bus": return 3.2f;
                case "truck": return 3.0f;
                case "auto": return 1.85f;
                case "two_wheeler": return 1.55f;
                case "bicycle": return 1.65f;
                case "pedestrian": return 1.7f;
                case "cattle": return 1.45f;
                case "pushcart": return 1.3f;
                case "static": return 1.3f;
                default: return 1.5f;
            }
        }

        public static readonly string[] Classes =
            { "car", "bus", "truck", "auto", "two_wheeler", "bicycle", "pedestrian", "cattle", "pushcart", "static" };

        /// A road user, origin at the centre of its footprint on the ground,
        /// facing +z. L and W are the footprint the simulation used.
        public static GameObject Agent(string cls, float L, float W)
        {
            Color c = Palette.OfClass(cls);
            Color dark = Palette.Mul(c, 0.45f);
            Color glass = Palette.Hex("#1e293b");
            float H = HeightOf(cls);
            var g = Root(cls, L, W, H, true);
            var t = g.transform;

            switch (cls)
            {
                case "bus":
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.75f, 0), new Vector3(W, 2.7f, L), c, name: "body");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 2.3f, 0.1f), new Vector3(W + 0.04f, 0.8f, L * 0.94f), glass, 0.8f, name: "windows");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 3.12f, 0), new Vector3(W * 0.9f, 0.1f, L * 0.95f), Palette.Mul(c, 1.25f), name: "roof");
                    for (int i = 0; i < 2; i++)
                    {
                        float z = (i == 0 ? 0.32f : -0.32f) * L;
                        Wheel(t, W / 2, z, 0.5f, 0.35f); Wheel(t, -W / 2, z, 0.5f, 0.35f);
                    }
                    break;

                case "truck":
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.35f, L / 2 - 1.1f), new Vector3(W, 1.9f, 2.2f), Palette.Mul(c, 1.2f), name: "cab");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.8f, L / 2 - 0.05f), new Vector3(W * 0.9f, 0.7f, 0.1f), glass, 0.8f, name: "windscreen");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.75f, -1.2f), new Vector3(W, 2.5f, L - 2.4f), c, name: "load");
                    Wheel(t, W / 2, L / 2 - 1.2f, 0.5f, 0.35f); Wheel(t, -W / 2, L / 2 - 1.2f, 0.5f, 0.35f);
                    Wheel(t, W / 2, -L / 2 + 1.4f, 0.5f, 0.35f); Wheel(t, -W / 2, -L / 2 + 1.4f, 0.5f, 0.35f);
                    break;

                case "auto":
                    Part(t, PrimitiveType.Cube, new Vector3(0, 0.6f, 0), new Vector3(W, 0.65f, L), c, name: "body");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.35f, -0.1f * L), new Vector3(W * 0.98f, 0.85f, L * 0.72f), Palette.Hex("#1f2937"), name: "cabin");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.79f, -0.1f * L), new Vector3(W, 0.08f, L * 0.78f), Palette.Hex("#111827"), name: "canopy");
                    Wheel(t, 0, L / 2 - 0.35f, 0.25f, 0.18f);
                    Wheel(t, W / 2 - 0.05f, -L / 2 + 0.45f, 0.25f, 0.18f);
                    Wheel(t, -W / 2 + 0.05f, -L / 2 + 0.45f, 0.25f, 0.18f);
                    break;

                case "two_wheeler":
                case "bicycle":
                {
                    bool bike = cls == "bicycle";
                    float fw = bike ? 0.07f : 0.28f;
                    Part(t, PrimitiveType.Cube, new Vector3(0, bike ? 0.65f : 0.6f, 0), new Vector3(fw, bike ? 0.12f : 0.45f, L * 0.7f), c, name: "frame");
                    var w1 = Wheel(t, 0, L / 2 - 0.33f, 0.33f, 0.08f); w1.transform.localScale = new Vector3(0.66f, 0.04f, 0.66f);
                    var w2 = Wheel(t, 0, -L / 2 + 0.33f, 0.33f, 0.08f); w2.transform.localScale = new Vector3(0.66f, 0.04f, 0.66f);
                    Part(t, PrimitiveType.Capsule, new Vector3(0, 1.15f, -0.05f), new Vector3(0.42f, 0.42f, 0.32f), dark, name: "rider");
                    Part(t, PrimitiveType.Sphere, new Vector3(0, 1.5f, 0.02f), Vector3.one * 0.26f, Palette.Hex("#d6b38a"), name: "head");
                    break;
                }

                case "pedestrian":
                    Part(t, PrimitiveType.Capsule, new Vector3(0, 0.72f, 0), new Vector3(0.42f, 0.72f, 0.3f), c, name: "body");
                    Part(t, PrimitiveType.Sphere, new Vector3(0, 1.55f, 0), Vector3.one * 0.26f, Palette.Hex("#d6b38a"), name: "head");
                    break;

                case "cattle":
                {
                    Color hide = Palette.Hex("#e7dccb");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.0f, -0.1f), new Vector3(W * 0.85f, 0.75f, L * 0.68f), hide, name: "body");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.32f, 0.12f * L), new Vector3(W * 0.55f, 0.35f, 0.4f), Palette.Mul(hide, 0.9f), name: "hump");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.22f, L / 2 - 0.25f), new Vector3(0.34f, 0.38f, 0.5f), c, name: "head");
                    Part(t, PrimitiveType.Cube, new Vector3(0.14f, 1.48f, L / 2 - 0.25f), new Vector3(0.05f, 0.2f, 0.05f), Palette.Hex("#d4d4d4"), name: "horn");
                    Part(t, PrimitiveType.Cube, new Vector3(-0.14f, 1.48f, L / 2 - 0.25f), new Vector3(0.05f, 0.2f, 0.05f), Palette.Hex("#d4d4d4"), name: "horn");
                    foreach (var p in new[] { new Vector2(0.25f, 0.55f), new Vector2(-0.25f, 0.55f), new Vector2(0.25f, -0.7f), new Vector2(-0.25f, -0.7f) })
                        Part(t, PrimitiveType.Cube, new Vector3(p.x * W, 0.32f, p.y * L / 2.2f), new Vector3(0.12f, 0.64f, 0.12f), Palette.Mul(hide, 0.75f), name: "leg");
                    break;
                }

                case "pushcart":
                    Part(t, PrimitiveType.Cube, new Vector3(0, 0.8f, 0), new Vector3(W, 0.12f, L), Palette.Hex("#6b4f2e"), name: "deck");
                    Part(t, PrimitiveType.Cube, new Vector3(0.2f, 1.0f, 0.3f), new Vector3(0.5f, 0.3f, 0.6f), Palette.Hex("#f97316"), name: "goods");
                    Part(t, PrimitiveType.Cube, new Vector3(-0.25f, 1.0f, -0.2f), new Vector3(0.5f, 0.3f, 0.7f), Palette.Hex("#84cc16"), name: "goods");
                    Part(t, PrimitiveType.Cube, new Vector3(0.1f, 1.18f, -0.4f), new Vector3(0.4f, 0.25f, 0.4f), c, name: "goods");
                    Wheel(t, W / 2, 0, 0.4f, 0.08f); Wheel(t, -W / 2, 0, 0.4f, 0.08f);
                    break;

                case "static":
                    Part(t, PrimitiveType.Cube, new Vector3(0, 0.6f, 0), new Vector3(W, 1.2f, L), c, name: "body");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.25f, 0), new Vector3(W * 1.05f, 0.08f, L * 1.05f), Palette.Hex("#f59e0b"), name: "stripe");
                    break;

                default: // car
                    Part(t, PrimitiveType.Cube, new Vector3(0, 0.62f, 0), new Vector3(W, 0.68f, L), c, name: "body");
                    Part(t, PrimitiveType.Cube, new Vector3(0, 1.2f, -0.06f * L), new Vector3(W * 0.84f, 0.5f, L * 0.5f), glass, 0.8f, name: "cabin");
                    Wheel(t, W / 2, L * 0.32f, 0.32f, 0.22f); Wheel(t, -W / 2, L * 0.32f, 0.32f, 0.22f);
                    Wheel(t, W / 2, -L * 0.32f, 0.32f, 0.22f); Wheel(t, -W / 2, -L * 0.32f, 0.32f, 0.22f);
                    break;
            }
            return g;
        }

        /// The ego: white hatchback with a roof LiDAR. No collider, so the
        /// simulated LiDAR does not see its own roof.
        public static GameObject Ego(float L, float W)
        {
            var g = new GameObject("Ego");
            var t = g.transform;
            Color body = Palette.Ego;
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.62f, 0), new Vector3(W, 0.66f, L), body, 0.55f, name: "body");
            Part(t, PrimitiveType.Cube, new Vector3(0, 1.13f, -0.05f * L), new Vector3(W * 0.86f, 0.4f, L * 0.54f), Palette.Hex("#0f172a"), 0.9f, name: "cabin");
            Part(t, PrimitiveType.Cube, new Vector3(0, 1.36f, -0.06f * L), new Vector3(W * 0.82f, 0.1f, L * 0.48f), body, 0.55f, name: "roof");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.75f, L / 2 + 0.01f), new Vector3(W * 0.8f, 0.08f, 0.02f), Palette.Plan, 0.5f, 3f, "light bar");
            Part(t, PrimitiveType.Cylinder, new Vector3(0, 1.5f, 0), new Vector3(0.32f, 0.08f, 0.32f), Palette.Hex("#111827"), 0.6f, name: "lidar base");
            Part(t, PrimitiveType.Cylinder, new Vector3(0, 1.62f, 0), new Vector3(0.26f, 0.05f, 0.26f), Palette.Hex("#22d3ee"), 0.6f, 2.5f, "lidar head");
            Wheel(t, W / 2, L * 0.32f, 0.32f, 0.22f); Wheel(t, -W / 2, L * 0.32f, 0.32f, 0.22f);
            Wheel(t, W / 2, -L * 0.32f, 0.32f, 0.22f); Wheel(t, -W / 2, -L * 0.32f, 0.32f, 0.22f);
            return g;
        }

        // ---- scenery (nominal sizes; instances are scaled) -------------------

        public const float HouseW = 6f, HouseD = 5.5f, HouseH = 3.4f;
        public const float TreeH = 6f, TreeR = 2f;
        public const float WallLength = 6f;

        public static GameObject House(Color wall, Color roof)
        {
            float w = HouseW, d = HouseD, h = HouseH;
            var g = Root("house", d, w, h + 1.2f, true);
            var t = g.transform;
            Part(t, PrimitiveType.Cube, new Vector3(0, h / 2, 0), new Vector3(w, h, d), wall, 0.1f, name: "walls");
            MeshPart(t, Prism(), new Vector3(0, h, 0), new Vector3(w + 0.4f, 1.2f, d + 0.4f), Lit(roof), "roof");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.95f, d / 2 + 0.01f), new Vector3(0.9f, 1.9f, 0.04f), Palette.Mul(wall, 0.45f), name: "door");
            Part(t, PrimitiveType.Cube, new Vector3(w * 0.28f, h * 0.6f, d / 2 + 0.01f), new Vector3(0.7f, 0.6f, 0.04f), Palette.Hex("#fde68a"), 0.3f, 1.2f, "window");
            return g;
        }

        public static GameObject Tree(Color leaf)
        {
            float h = TreeH, r = TreeR;
            var g = new GameObject("tree");
            var cap = g.AddComponent<CapsuleCollider>();
            cap.center = new Vector3(0, h * 0.6f, 0); cap.height = h; cap.radius = r * 0.7f;
            var t = g.transform;
            Part(t, PrimitiveType.Cylinder, new Vector3(0, h * 0.3f, 0), new Vector3(0.25f, h * 0.3f, 0.25f), Palette.Hex("#4a3424"), name: "trunk");
            Part(t, PrimitiveType.Sphere, new Vector3(0, h * 0.72f, 0), new Vector3(2 * r, 1.5f * r, 2 * r), leaf, 0.05f, name: "crown");
            Part(t, PrimitiveType.Sphere, new Vector3(r * 0.3f, h * 0.9f, -r * 0.2f), new Vector3(1.3f * r, 1.1f * r, 1.3f * r), Palette.Mul(leaf, 1.15f), 0.05f, name: "crown top");
            return g;
        }

        public static GameObject Pole()
        {
            var g = new GameObject("pole");
            var c = g.AddComponent<CapsuleCollider>();
            c.center = new Vector3(0, 3.5f, 0); c.height = 7f; c.radius = 0.15f;
            Part(g.transform, PrimitiveType.Cylinder, new Vector3(0, 3.5f, 0), new Vector3(0.18f, 3.5f, 0.18f), Palette.Hex("#6b7280"), name: "post");
            Part(g.transform, PrimitiveType.Cube, new Vector3(0, 6.6f, 0), new Vector3(1.4f, 0.08f, 0.08f), Palette.Hex("#6b7280"), name: "arm");
            return g;
        }

        public static GameObject Wall(Color c)
        {
            var g = Root("wall", WallLength, 0.3f, 1.1f, true);
            Part(g.transform, PrimitiveType.Cube, new Vector3(0, 0.55f, 0), new Vector3(0.3f, 1.1f, WallLength), c, 0.05f, name: "wall");
            return g;
        }

        // ---- road-side objects (V4) ------------------------------------------
        // Off the carriageway, so they are static structure for the sensors'
        // map, never obstacles in the stack's ground truth.

        /// A flat field patch, 1 x 1 m (scaled by the generator). No collider:
        /// it is ground.
        public static GameObject Field(Color c)
        {
            var g = new GameObject("field");
            var f = Part(g.transform, PrimitiveType.Cube, new Vector3(0, 0.01f, 0), new Vector3(1f, 0.02f, 1f), c, 0.02f, name: "patch");
            f.GetComponent<MeshRenderer>().shadowCastingMode = ShadowCastingMode.Off;
            return g;
        }

        public static GameObject Cone()
        {
            var g = new GameObject("cone");
            var c = g.AddComponent<CapsuleCollider>();
            c.center = new Vector3(0, 0.35f, 0); c.height = 0.7f; c.radius = 0.2f;
            Part(g.transform, PrimitiveType.Cube, new Vector3(0, 0.02f, 0), new Vector3(0.42f, 0.04f, 0.42f), Palette.Hex("#1f2937"), name: "base");
            Part(g.transform, PrimitiveType.Cylinder, new Vector3(0, 0.36f, 0), new Vector3(0.26f, 0.32f, 0.26f), Palette.Hex("#f97316"), 0.3f, 0.4f, "body");
            Part(g.transform, PrimitiveType.Cylinder, new Vector3(0, 0.42f, 0), new Vector3(0.27f, 0.05f, 0.27f), Palette.Hex("#f8fafc"), 0.4f, 0.6f, "band");
            return g;
        }

        public static GameObject Barrier()
        {
            var g = Root("barrier", 2.0f, 0.4f, 1.0f, true);
            var t = g.transform;
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.75f, 0), new Vector3(0.12f, 0.3f, 2.0f), Palette.Hex("#ef4444"), 0.3f, 0.3f, "rail");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.75f, -0.5f), new Vector3(0.13f, 0.3f, 0.35f), Palette.Hex("#f8fafc"), 0.3f, 0.4f, "stripe");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.75f, 0.5f), new Vector3(0.13f, 0.3f, 0.35f), Palette.Hex("#f8fafc"), 0.3f, 0.4f, "stripe");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.3f, -0.85f), new Vector3(0.4f, 0.6f, 0.1f), Palette.Hex("#374151"), name: "leg");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.3f, 0.85f), new Vector3(0.4f, 0.6f, 0.1f), Palette.Hex("#374151"), name: "leg");
            return g;
        }

        public static GameObject Sign()
        {
            var g = Root("sign", 0.1f, 0.9f, 2.4f, true);
            var t = g.transform;
            Part(t, PrimitiveType.Cylinder, new Vector3(0, 1.0f, 0), new Vector3(0.08f, 1.0f, 0.08f), Palette.Hex("#9ca3af"), name: "post");
            Part(t, PrimitiveType.Cube, new Vector3(0, 2.0f, 0.05f), new Vector3(0.85f, 0.7f, 0.04f), Palette.Hex("#1d4ed8"), 0.3f, 0.3f, "plate");
            Part(t, PrimitiveType.Cube, new Vector3(0, 2.0f, 0.075f), new Vector3(0.7f, 0.08f, 0.01f), Palette.Hex("#f8fafc"), 0.3f, 0.5f, "arrow");
            return g;
        }

        public static GameObject Milestone()
        {
            var g = Root("milestone", 0.25f, 0.4f, 0.8f, true);
            var t = g.transform;
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.3f, 0), new Vector3(0.4f, 0.6f, 0.25f), Palette.Hex("#f1f5f9"), 0.2f, name: "stone");
            Part(t, PrimitiveType.Cube, new Vector3(0, 0.68f, 0), new Vector3(0.4f, 0.16f, 0.25f), Palette.Hex("#facc15"), 0.2f, 0.3f, "cap");
            return g;
        }

        public static readonly Color[] WallColors =
        {
            Palette.Hex("#c9b79c"), Palette.Hex("#d6c7a1"), Palette.Hex("#b9a48a"),
            Palette.Hex("#9fb3a8"), Palette.Hex("#c7a7a0"), Palette.Hex("#a9b8c9"),
        };
        public static readonly Color[] RoofColors = { Palette.Hex("#8c4a3a"), Palette.Hex("#7a5c3e"), Palette.Hex("#5e6b74"), Palette.Hex("#9b5b45") };
        public static readonly Color[] LeafColors = { Palette.Hex("#3f6b3a"), Palette.Hex("#4c7a3d"), Palette.Hex("#35603a"), Palette.Hex("#5a7f3a") };
    }
}
