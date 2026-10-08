// Colours, mirrored from tools/sihviz.py so the 3D view, the 2D previews and
// the submission video all describe a rickshaw or a planned path the same way.

using System.Collections.Generic;
using UnityEngine;

namespace Sih
{
    public static class Palette
    {
        /// "#rrggbb" or "#rrggbbaa". Plain C#, so it is safe in field
        /// initialisers of components (Unity's own parser is not).
        public static Color Hex(string h)
        {
            if (string.IsNullOrEmpty(h)) return Color.magenta;
            if (h[0] == '#') h = h.Substring(1);
            if (h.Length != 6 && h.Length != 8) return Color.magenta;
            uint v = System.Convert.ToUInt32(h, 16);
            if (h.Length == 6) v = (v << 8) | 0xFF;
            return new Color(((v >> 24) & 255) / 255f, ((v >> 16) & 255) / 255f, ((v >> 8) & 255) / 255f, (v & 255) / 255f);
        }

        public static readonly Color Bg     = Hex("#0c0f14");
        public static readonly Color Panel  = Hex("#141922");
        public static readonly Color Line   = Hex("#2a3446");
        public static readonly Color Ink    = Hex("#e8edf5");
        public static readonly Color Dim    = Hex("#93a1b5");
        public static readonly Color Road   = Hex("#39465c");
        public static readonly Color Centre = Hex("#41506b");
        public static readonly Color Ground = Hex("#161c24");

        public static readonly Color Ego    = Hex("#f5f7fa");
        public static readonly Color Plan   = Hex("#22d3aa");
        public static readonly Color Track  = Hex("#60a5fa");
        public static readonly Color Goal   = Hex("#34d399");
        public static readonly Color Look   = Hex("#f472b6");

        public static readonly Dictionary<string, Color> State = new Dictionary<string, Color>
        {
            { "CRUISE", Hex("#34d399") }, { "FOLLOW", Hex("#60a5fa") }, { "NUDGE", Hex("#fbbf24") },
            { "YIELD",  Hex("#fb923c") }, { "CREEP",  Hex("#f87171") }, { "STOP",  Hex("#ef4444") },
            { "REVERSE", Hex("#c084fc") },
        };

        public static readonly Dictionary<string, Color> Class = new Dictionary<string, Color>
        {
            { "car",         Hex("#5b8fd6") }, { "bus",        Hex("#3b63a8") },
            { "truck",       Hex("#4a6fa0") }, { "auto",       Hex("#f2c744") },
            { "two_wheeler", Hex("#e4813b") }, { "bicycle",    Hex("#e8a33d") },
            { "pedestrian",  Hex("#ef5d75") }, { "cattle",     Hex("#b07a4a") },
            { "pushcart",    Hex("#9a72d0") }, { "static",     Hex("#7b8698") },
        };

        public static readonly Dictionary<string, string> ClassLabel = new Dictionary<string, string>
        {
            { "car", "car" }, { "bus", "bus" }, { "truck", "truck" }, { "auto", "auto-rickshaw" },
            { "two_wheeler", "two-wheeler" }, { "bicycle", "bicycle" }, { "pedestrian", "pedestrian" },
            { "cattle", "cattle" }, { "pushcart", "pushcart" }, { "static", "obstacle" },
        };

        // Sensors: camera, radar, lidar, in the order the export codes them.
        public static readonly Color[] Sensor = { Hex("#facc15"), Hex("#e879f9"), Hex("#22d3ee") };
        public static readonly string[] SensorName = { "camera", "radar", "lidar" };

        // Candidate status, in the order sih_lattice_plan codes it.
        public static readonly string[] CandCode =
            { "ok", "risk", "conflict", "slope", "curvature", "speed", "lat. accel" };
        public static readonly Color[] CandReject =
        {
            Hex("#22d3aa"), Hex("#f87171"), Hex("#ef4444"), Hex("#64748b"),
            Hex("#94a3b8"), Hex("#a78bfa"), Hex("#c084fc"),
        };

        public static Color OfClass(string c) =>
            c != null && Class.TryGetValue(c, out var v) ? v : Hex("#8a94a6");

        public static Color OfState(string s) =>
            s != null && State.TryGetValue(s, out var v) ? v : Dim;

        public static Color A(Color c, float a) { c.a = a; return c; }

        public static Color Mul(Color c, float k) => new Color(c.r * k, c.g * k, c.b * k, c.a);
    }
}
