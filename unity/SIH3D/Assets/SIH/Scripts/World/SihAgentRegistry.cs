// The road users in the scene: one object per road user in the run, made
// from the prefab catalog by class when the run first mentions it -- the
// scripted traffic of a run file, the random traffic the stack draws, or a
// road user a judge drops in. The replay poses them; the sensors see them.
//
// Each is scaled to its class's footprint as the stack has it, so what the
// sensors hit is the size the stack's ground truth says. In edit mode,
// Preview from run file shows the run file's road users where they start.

using System.Collections.Generic;
using UnityEngine;

namespace Sih
{
    public class SihAgentRegistry : MonoBehaviour
    {
        public SihCatalog catalog;
        [Tooltip("Scale each one so its collider matches the class footprint the stack uses.")]
        public bool fitToClassSize = true;

        [Header("Live (read only)")]
        public int count;
        public int seed;

        readonly Dictionary<int, SihAgent> byId = new Dictionary<int, SihAgent>();
        RunData run;
        System.Random rng = new System.Random(1);

        public IEnumerable<SihAgent> All => byId.Values;

        /// A new run: everything spawned for the last one goes.
        public void Begin(RunData r, int worldSeed)
        {
            Clear();
            run = r;
            seed = worldSeed;
            rng = new System.Random(worldSeed ^ 0x5eed);
        }

        public void Clear()
        {
            byId.Clear();
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                var c = transform.GetChild(i).gameObject;
                if (Application.isPlaying) Destroy(c); else DestroyImmediate(c);
            }
            count = 0;
        }

        /// The object for road user id, made on first use. Starts inactive;
        /// the replay shows it when the run has it in view.
        public SihAgent Get(int id, string cls)
        {
            if (byId.TryGetValue(id, out var a) && a != null) return a;
            if (catalog == null) return null;
            var prefab = catalog.RoadUser(cls, rng);
            if (prefab == null) return null;
            var go = Instantiate(prefab, transform);
            go.name = $"{cls} #{id}";
            a = go.GetComponent<SihAgent>() ?? go.AddComponent<SihAgent>();
            a.agentId = id;
            a.agentClass = cls;
            if (fitToClassSize && run != null) Fit(go, cls);
            go.SetActive(false);
            byId[id] = a;
            count = byId.Count;
            return a;
        }

        void Fit(GameObject go, string cls)
        {
            var col = go.GetComponent<BoxCollider>();
            if (col == null) return;
            var want = run.SizeOf(cls);                     // length, width
            // The collider's size is in the prefab's own units: width across
            // (x), length along (z).
            var s = go.transform.localScale;
            go.transform.localScale = new Vector3(want.y / Mathf.Max(col.size.x, 0.01f), s.y, want.x / Mathf.Max(col.size.z, 0.01f));
        }

        /// Edit mode: the run file's road users where they first appear.
        public void PreviewFrom(RunData r)
        {
            Begin(r, 26037);
            foreach (var f in r.frames)
                for (int k = 0; k < f.agentId.Length; k++)
                {
                    if (byId.ContainsKey(f.agentId[k])) continue;
                    var a = Get(f.agentId[k], k < f.agentClass.Length ? f.agentClass[k] : "car");
                    if (a == null) continue;
                    a.transform.SetPositionAndRotation(Frames.Pos(f.agents[3 * k], f.agents[3 * k + 1]), Frames.Rot(f.agents[3 * k + 2]));
                    a.gameObject.SetActive(true);
                }
        }
    }
}
