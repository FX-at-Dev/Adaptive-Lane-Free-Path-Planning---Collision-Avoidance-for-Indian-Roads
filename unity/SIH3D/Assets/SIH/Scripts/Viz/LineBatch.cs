// Thick lines expanded to width on the GPU (SihLine.shader).
//
// Each vertex carries its point, the line direction there, which side of the
// line it is on, the width and whether the ribbon lies flat on the ground or
// turns to face the camera.
//
// LineBuilder only collects geometry and writes it into a mesh. The editor
// uses it to bake the road markings into mesh assets; LineBatch, a component
// on an overlay object in the scene, uses it to refill that object's mesh
// whenever the planner's snapshot changes. The overlay's look (material,
// glow, draw order) is on its MeshRenderer, so it can be changed in the scene.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Sih
{
    public class LineBuilder
    {
        readonly List<Vector3> pos = new List<Vector3>();
        readonly List<Vector4> dir = new List<Vector4>();   // xyz direction, w side
        readonly List<Vector4> prm = new List<Vector4>();   // x width, y flat
        readonly List<Color> col = new List<Color>();
        readonly List<int> idx = new List<int>();
        float maxWidth;

        public int VertexCount => pos.Count;

        public void Clear()
        {
            pos.Clear(); dir.Clear(); prm.Clear(); col.Clear(); idx.Clear();
            maxWidth = 0f;
        }

        void Pair(Vector3 p, Vector3 d, float w, bool flat, Color c)
        {
            float f = flat ? 1f : 0f;
            pos.Add(p); dir.Add(new Vector4(d.x, d.y, d.z, -1f)); prm.Add(new Vector4(w, f, 0, 0)); col.Add(c);
            pos.Add(p); dir.Add(new Vector4(d.x, d.y, d.z, +1f)); prm.Add(new Vector4(w, f, 0, 0)); col.Add(c);
            if (w > maxWidth) maxWidth = w;
        }

        void Quad(int a)
        {
            // a, a+1 one end; a+2, a+3 the other.
            idx.Add(a); idx.Add(a + 2); idx.Add(a + 1);
            idx.Add(a + 1); idx.Add(a + 2); idx.Add(a + 3);
        }

        public void Seg(Vector3 a, Vector3 b, float w, Color c, bool flat = false) => Seg(a, b, w, c, c, flat);

        public void Seg(Vector3 a, Vector3 b, float w, Color ca, Color cb, bool flat = false)
        {
            var d = b - a;
            if (d.sqrMagnitude < 1e-10f) return;
            int k = pos.Count;
            Pair(a, d, w, flat, ca);
            Pair(b, d, w, flat, cb);
            Quad(k);
        }

        /// A joined polyline; per-point colours and widths optional.
        public void Strip(IList<Vector3> p, float w, Color c, bool flat = false, bool closed = false,
                          IList<Color> cs = null, IList<float> ws = null)
        {
            int n = p.Count;
            if (n < 2) return;
            int k = pos.Count;
            for (int i = 0; i < n; i++)
            {
                Vector3 prev = closed ? p[(i - 1 + n) % n] : p[Mathf.Max(i - 1, 0)];
                Vector3 next = closed ? p[(i + 1) % n] : p[Mathf.Min(i + 1, n - 1)];
                Vector3 d = next - prev;
                if (d.sqrMagnitude < 1e-10f) d = i > 0 ? p[i] - p[i - 1] : Vector3.forward;
                Pair(p[i], d, ws != null ? ws[i] : w, flat, cs != null ? cs[i] : c);
            }
            for (int i = 0; i < n - 1; i++) Quad(k + 2 * i);
            if (closed)
            {
                int a = k + 2 * (n - 1);
                idx.Add(a); idx.Add(k); idx.Add(a + 1);
                idx.Add(a + 1); idx.Add(k); idx.Add(k + 1);
            }
        }

        public void Circle(Vector3 c, float r, float w, Color col, int seg = 32)
        {
            var p = new Vector3[seg];
            for (int i = 0; i < seg; i++)
            {
                float a = i * Mathf.PI * 2f / seg;
                p[i] = c + new Vector3(Mathf.Cos(a) * r, 0, Mathf.Sin(a) * r);
            }
            Strip(p, w, col, true, true);
        }

        /// Flat filled disc, drawn as a ribbon of width r around radius r/2.
        public void Dot(Vector3 c, float r, Color col, int seg = 16) => Circle(c, r * 0.5f, r, col, seg);

        public void WriteTo(Mesh mesh)
        {
            mesh.Clear();
            if (pos.Count == 0) return;
            mesh.indexFormat = IndexFormat.UInt32;
            mesh.SetVertices(pos);
            mesh.SetUVs(0, dir);
            mesh.SetUVs(1, prm);
            mesh.SetColors(col);
            mesh.SetIndices(idx, MeshTopology.Triangles, 0, false);
            mesh.RecalculateBounds();
            var b = mesh.bounds;
            b.Expand(maxWidth * 2f + 1f);
            mesh.bounds = b;
        }
    }

    /// One overlay layer in the scene. Its mesh is refilled from the run while
    /// playing (or while previewing in the editor) and is never saved.
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class LineBatch : MonoBehaviour
    {
        readonly LineBuilder lines = new LineBuilder();
        Mesh mesh;

        public void Clear() => lines.Clear();
        public void Seg(Vector3 a, Vector3 b, float w, Color c, bool flat = false) => lines.Seg(a, b, w, c, flat);
        public void Seg(Vector3 a, Vector3 b, float w, Color ca, Color cb, bool flat = false) => lines.Seg(a, b, w, ca, cb, flat);
        public void Strip(IList<Vector3> p, float w, Color c, bool flat = false, bool closed = false,
                          IList<Color> cs = null, IList<float> ws = null) => lines.Strip(p, w, c, flat, closed, cs, ws);
        public void Circle(Vector3 c, float r, float w, Color col, int seg = 32) => lines.Circle(c, r, w, col, seg);
        public void Dot(Vector3 c, float r, Color col, int seg = 16) => lines.Dot(c, r, col, seg);

        public void Flush()
        {
            var mf = GetComponent<MeshFilter>();
            if (mesh == null)
            {
                // After a script reload in the editor, pick the live mesh back up.
                var cur = mf.sharedMesh;
                if (cur != null && (cur.hideFlags & HideFlags.DontSave) != 0) mesh = cur;
                else
                {
                    mesh = new Mesh { name = name + " (live)", hideFlags = HideFlags.DontSave };
                    mesh.MarkDynamic();
                }
            }
            if (mf.sharedMesh != mesh) mf.sharedMesh = mesh;
            lines.WriteTo(mesh);
        }

        public void SetVisible(bool v)
        {
            var r = GetComponent<MeshRenderer>();
            if (r.enabled != v) r.enabled = v;
        }

        void OnDestroy()
        {
            if (mesh == null) return;
            if (Application.isPlaying) Destroy(mesh);
            else DestroyImmediate(mesh);
        }
    }
}
