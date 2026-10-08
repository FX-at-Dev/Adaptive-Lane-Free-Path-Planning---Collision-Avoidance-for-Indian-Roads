// The prefab catalog: for every road-user class and every kind of scenery,
// the prefabs that can stand in for it and how often each is picked.
//
// Everything the world generators place comes from here, so changing how a
// car, a tree or a house looks is a matter of editing its prefab or adding
// another variant to its list -- nothing in code. The asset is
// Assets/SIH/Catalog.asset; SIH > Create Scenes fills it with the stylised
// prefabs and never overwrites lists you have edited.

using System;
using UnityEngine;

namespace Sih
{
    [CreateAssetMenu(menuName = "SIH/Catalog", fileName = "Catalog")]
    public class SihCatalog : ScriptableObject
    {
        [Serializable]
        public class Variant
        {
            public GameObject prefab;
            [Min(0f)] public float weight = 1f;
        }

        [Serializable]
        public class Entry
        {
            [Tooltip("Road users: the class (car, bus, truck, auto, two_wheeler, bicycle, pedestrian, cattle, pushcart, static). Scenery: the kind (house, tree, wall, pole, field, cone, barrier, sign, milestone).")]
            public string key;
            public Variant[] variants = new Variant[0];
        }

        [Tooltip("One entry per road-user class.")]
        public Entry[] roadUsers = new Entry[0];
        [Tooltip("One entry per kind of scenery and road object.")]
        public Entry[] scenery = new Entry[0];
        [Tooltip("Used for a road-user class with no entry.")]
        public GameObject fallbackRoadUser;

        public GameObject RoadUser(string cls, System.Random rng) => Pick(roadUsers, cls, rng) ?? fallbackRoadUser;
        public GameObject Scenery(string kind, System.Random rng) => Pick(scenery, kind, rng);

        /// A weighted random variant of key, or null if there is none.
        public static GameObject Pick(Entry[] list, string key, System.Random rng)
        {
            if (list == null) return null;
            foreach (var e in list)
            {
                if (e == null || e.key != key || e.variants == null) continue;
                float total = 0f;
                foreach (var v in e.variants) if (v != null && v.prefab != null) total += Mathf.Max(0f, v.weight);
                if (total <= 0f) return null;
                double r = rng.NextDouble() * total;
                foreach (var v in e.variants)
                {
                    if (v == null || v.prefab == null) continue;
                    r -= Mathf.Max(0f, v.weight);
                    if (r <= 0) return v.prefab;
                }
                foreach (var v in e.variants) if (v != null && v.prefab != null) return v.prefab;
            }
            return null;
        }
    }
}
