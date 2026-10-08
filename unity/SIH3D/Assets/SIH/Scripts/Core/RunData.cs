// A completed run, as written by sim/sih_export_run.m.
//
// Frames are every 0.1 s (each planner cycle); the ego series is every 0.05 s.
// The viewer interpolates both, so playback is smooth at any rate while every
// number it shows is one the simulation actually produced.

using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Sih
{
    /// One tracked object as the stack saw it: its footprint, the geometry
    /// every decision used (core/world/sih_footprint.m).
    public struct TrackView
    {
        public int id;
        public string cls;
        /// Footprint centre, world frame (Octave x, y).
        public float x, y;
        /// Motion heading of the filter and speed (0 when still).
        public float psi, v;
        public float pxx, pxy, pyy;
        /// 0 tentative, 1 confirmed, 2 coasting.
        public int status;
        /// Footprint length, width and long-axis orientation; L 0 in runs
        /// written before footprints were exported.
        public float L, W, theta;
        public bool still, inPath;
        public bool HasFootprint => L > 0f;
    }

    /// One detection: where a sensor put an object, and the box it saw.
    public struct DetView
    {
        public float x, y;
        /// 0 camera, 1 radar, 2 lidar.
        public int sensor;
        public float L, W, theta;
        public bool HasBox => L > 0f;
    }

    public class Think
    {
        public const int PredStride = 4;    // src mode w r
        public const int CandStride = 3;    // code cost risk

        public TrackView[] tracks = new TrackView[0];
        public DetView[] dets = new DetView[0];
        public float[] pred, predXY;
        public int predPts;
        public float[] cand, candXY;
        public int candPts;
        public float[] ra;      // ttc lead_id lead_gap lead_speed min_clear cross_ttc cross_id n_near free_left free_right
        public float[] bp;      // v_cap d_max risk_tol
        public string reason;
        public float[] info;    // n_eval n_feas rej(lon slope curv speed alat risk) d_limit latency feasible
        public Vector2 look;

        public int NTracks => tracks.Length;
        public int NDets => dets.Length;
        public int NPred => pred.Length / PredStride;
        public int NCand => cand.Length / CandStride;

        public float Ra(int i) => ra != null && ra.Length > i ? ra[i] : 0f;
        public float Info(int i) => info != null && info.Length > i ? info[i] : 0f;
    }

    public class Frame
    {
        public float t;
        public string state;
        public float ex, ey, epsi, ev;
        public float[] agents;       // stride 3: x y psi
        public string[] agentClass;
        public int[] agentId;
        public float[] tracks;       // stride 4: x y psi v (confirmed + coasting)
        public float[] trajX, trajY;
        public Think think;

        public int NAgents => agents.Length / 3;
    }

    public class RunData
    {
        public string path, name, desc;
        public float dt, replanDt, vMax, egoL, egoW, rearToCentre = 1.35f, latencyBudget, tEnd, wheelbase = 2.7f;
        public Vector2 goal;
        public bool hasThink;
        public readonly Dictionary<string, Vector2> classSize = new Dictionary<string, Vector2>();
        public float lidarRange = 40f, lidarMount = 1.9f, lidarVLo = -15f, lidarVHi = 15f;
        public int lidarBeams = 32, lidarBins = 1024;

        public float[] roadX, roadY, roadHw, roadPsi;
        /// Cross streets: station along the road, half width; flattened.
        public float[] crossStreets = new float[0];

        public float[] sT, sX, sY, sPsi, sV, sA, sDelta, sMinClear, sVCap, sRisk, sTracks;
        public string[] sState;
        public float[] latT, latMs;

        public readonly List<Frame> frames = new List<Frame>();

        public bool reached, collided;
        public float tReached, minClear;

        /// True for a run arriving step by step from the live co-simulation.
        public bool live;

        public float T0 => sT.Length > 0 ? sT[0] : 0f;
        public float T1 => sT.Length > 0 ? sT[sT.Length - 1] : 0f;

        public static RunData Load(string path)
        {
            var r = Parse(File.ReadAllText(path));
            r.path = path;
            return r;
        }

        /// From the text of a run file (a TextAsset in the scene, or a file).
        public static RunData Parse(string json) => FromJson(Json.Parse(json));

        /// A run that is about to happen: the meta and road of the live
        /// co-simulation's HELLO, with no steps yet. AppendStep and
        /// AppendFrame add them as they arrive.
        public static RunData FromHeader(object run)
        {
            var r = FromJson(run);
            r.live = true;
            return r;
        }

        static RunData FromJson(object root)
        {
            var r = new RunData();

            var meta = Json.Get(root, "meta");
            r.name = Json.S(meta, "name");
            r.desc = Json.S(meta, "desc");
            r.dt = (float)Json.D(meta, "dt", 0.05);
            r.replanDt = (float)Json.D(meta, "replan_dt", 0.1);
            r.vMax = (float)Json.D(meta, "v_max", 8);
            r.egoL = (float)Json.D(meta, "ego_length", 4);
            r.egoW = (float)Json.D(meta, "ego_width", 1.8);
            r.rearToCentre = (float)Json.D(meta, "rear_axle_to_centre", 1.35);
            r.latencyBudget = (float)Json.D(meta, "latency_budget", 100);
            r.tEnd = (float)Json.D(meta, "t_end", 0);
            r.wheelbase = (float)Json.D(meta, "wheelbase", 2.7);
            r.crossStreets = Json.F(meta, "cross_streets");
            var g = Json.F(meta, "goal");
            r.goal = new Vector2(g[0], g[1]);
            r.hasThink = Json.D(meta, "think") > 0.5;

            if (Json.Get(meta, "class_size") is Dictionary<string, object> cs)
                foreach (var kv in cs)
                {
                    var lw = Json.F(kv.Value);
                    r.classSize[kv.Key] = new Vector2(lw[0], lw[1]);
                }

            var lidar = Json.Get(Json.Get(meta, "sensors"), "lidar");
            if (lidar != null)
            {
                r.lidarRange = (float)Json.D(lidar, "range", 40);
                r.lidarMount = (float)Json.D(lidar, "mount_height", 1.9);
                r.lidarBeams = (int)Json.D(lidar, "beams", 32);
                r.lidarBins = (int)Json.D(lidar, "azimuth_bins", 1024);
                var vf = Json.F(lidar, "v_fov");
                if (vf.Length == 2) { r.lidarVLo = vf[0] * Mathf.Rad2Deg; r.lidarVHi = vf[1] * Mathf.Rad2Deg; }
            }

            var road = Json.Get(root, "road");
            r.roadX = Json.F(road, "x");
            r.roadY = Json.F(road, "y");
            r.roadHw = Json.F(road, "hw");
            r.roadPsi = Json.F(road, "psi");
            if (r.roadPsi.Length != r.roadX.Length) r.roadPsi = DerivePsi(r.roadX, r.roadY);

            var s = Json.Get(root, "series");
            r.sT = Json.F(s, "t"); r.sX = Json.F(s, "x"); r.sY = Json.F(s, "y");
            r.sPsi = Json.F(s, "psi"); r.sV = Json.F(s, "v"); r.sA = Json.F(s, "a");
            r.sDelta = Json.F(s, "delta"); r.sMinClear = Json.F(s, "min_clear");
            r.sVCap = Json.F(s, "v_cap"); r.sRisk = Json.F(s, "plan_risk");
            r.sTracks = Json.F(s, "n_tracks");
            r.sState = Json.Strs(Json.Get(s, "state"));
            r.latT = Json.F(s, "latency_t"); r.latMs = Json.F(s, "latency_ms");

            if (Json.Get(root, "frames") is List<object> fl)
                foreach (var fo in fl) r.frames.Add(ReadFrame(fo));

            var res = Json.Get(root, "result");
            r.reached = Json.D(res, "reached") > 0.5;
            r.collided = Json.D(res, "collided") > 0.5;
            r.tReached = (float)Json.D(res, "t_reached", double.NaN);
            r.minClear = (float)Json.D(res, "min_clear");
            return r;
        }

        // ---- live --------------------------------------------------------------

        /// One 50 ms step of a live run: the ego as the plant moved it, and
        /// what the stack measured and decided (the ACT's series).
        public void AppendStep(float t, float x, float y, float psi, float v, float a, float delta, object series)
        {
            Add(ref sT, t); Add(ref sX, x); Add(ref sY, y); Add(ref sPsi, psi);
            Add(ref sV, v); Add(ref sA, a); Add(ref sDelta, delta);
            Add(ref sMinClear, (float)Json.D(series, "min_clear", 999));
            Add(ref sVCap, (float)Json.D(series, "v_cap"));
            Add(ref sRisk, (float)Json.D(series, "plan_risk"));
            Add(ref sTracks, (float)Json.D(series, "n_tracks"));
            var st = Json.S(series, "state", "CRUISE");
            System.Array.Resize(ref sState, sState.Length + 1);
            sState[sState.Length - 1] = st;
            float lat = (float)Json.D(series, "latency_ms", -1);
            if (lat >= 0) { Add(ref latT, t); Add(ref latMs, lat); }
            minClear = sMinClear.Length == 1 ? sMinClear[0] : Mathf.Min(minClear, sMinClear[sMinClear.Length - 1]);
        }

        /// A planner-cycle snapshot (run-file frame form) of a live run.
        public void AppendFrame(object frame) => frames.Add(ReadFrame(frame));

        static void Add(ref float[] arr, float v)
        {
            System.Array.Resize(ref arr, arr.Length + 1);
            arr[arr.Length - 1] = v;
        }

        static Frame ReadFrame(object fo)
        {
            var f = new Frame
            {
                t = (float)Json.D(fo, "t"),
                state = Json.S(fo, "state", "CRUISE"),
                agents = Json.F(fo, "agents"),
                agentClass = Json.Strs(Json.Get(fo, "agent_class")),
                tracks = Json.F(fo, "tracks"),
            };
            var e = Json.F(fo, "ego");
            f.ex = e[0]; f.ey = e[1]; f.epsi = e[2]; f.ev = e[3];

            var ids = Json.F(fo, "agent_id");
            f.agentId = new int[ids.Length];
            for (int i = 0; i < ids.Length; i++) f.agentId[i] = Mathf.RoundToInt(ids[i]);

            var tj = Json.Rows(Json.Get(fo, "traj"));
            f.trajX = tj.Count == 2 ? tj[0] : new float[0];
            f.trajY = tj.Count == 2 ? tj[1] : new float[0];

            var th = Json.Get(fo, "think");
            if (th != null)
            {
                var k = new Think
                {
                    tracks = ReadTracks(th),
                    dets = ReadDets(th),
                    pred = Json.F(th, "pred"),
                    predXY = Json.F(th, "pred_xy"),
                    predPts = (int)Json.D(th, "pred_pts"),
                    cand = Json.F(th, "cand"),
                    candXY = Json.F(th, "cand_xy"),
                    candPts = (int)Json.D(th, "cand_pts"),
                    ra = Json.F(th, "ra"),
                    bp = Json.F(th, "bp"),
                    reason = Json.S(th, "reason"),
                    info = Json.F(th, "info"),
                };
                var lk = Json.F(th, "look");
                k.look = lk.Length == 2 ? new Vector2(lk[0], lk[1]) : new Vector2(f.ex, f.ey);
                f.think = k;
            }
            return f;
        }

        /// Tracks, whatever the stride: 9 values in older runs (id x y psi v
        /// Pxx Pxy Pyy status), 14 with the footprint (+ L W theta still
        /// in_path).
        static TrackView[] ReadTracks(object th)
        {
            var a = Json.F(th, "tracks");
            var cls = Json.Strs(Json.Get(th, "track_class"));
            int stride = Mathf.Max(9, (int)Json.D(th, "track_stride", 9));
            int n = a.Length / stride;
            var r = new TrackView[n];
            for (int i = 0; i < n; i++)
            {
                int o = stride * i;
                var t = new TrackView
                {
                    id = Mathf.RoundToInt(a[o]), x = a[o + 1], y = a[o + 2], psi = a[o + 3], v = a[o + 4],
                    pxx = a[o + 5], pxy = a[o + 6], pyy = a[o + 7], status = Mathf.RoundToInt(a[o + 8]),
                    cls = i < cls.Length ? cls[i] : "car",
                };
                if (stride >= 14)
                {
                    t.L = a[o + 9]; t.W = a[o + 10]; t.theta = a[o + 11];
                    t.still = a[o + 12] > 0.5f; t.inPath = a[o + 13] > 0.5f;
                }
                r[i] = t;
            }
            return r;
        }

        /// Detections: 3 values in older runs (x y sensor), 6 with the box.
        static DetView[] ReadDets(object th)
        {
            var a = Json.F(th, "dets");
            int stride = Mathf.Max(3, (int)Json.D(th, "det_stride", 3));
            int n = a.Length / stride;
            var r = new DetView[n];
            for (int i = 0; i < n; i++)
            {
                int o = stride * i;
                var d = new DetView { x = a[o], y = a[o + 1], sensor = Mathf.Clamp(Mathf.RoundToInt(a[o + 2]), 0, 2) };
                if (stride >= 6 && a[o + 3] > 0f) { d.L = a[o + 3]; d.W = a[o + 4]; d.theta = a[o + 5]; }
                r[i] = d;
            }
            return r;
        }

        static float[] DerivePsi(float[] x, float[] y)
        {
            var p = new float[x.Length];
            for (int i = 0; i < x.Length; i++)
            {
                int a = Mathf.Max(0, i - 1), b = Mathf.Min(x.Length - 1, i + 1);
                p[i] = Mathf.Atan2(y[b] - y[a], x[b] - x[a]);
            }
            return p;
        }

        public Vector2 SizeOf(string cls)
        {
            if (cls != null && classSize.TryGetValue(cls, out var v)) return v;
            switch (cls)
            {
                case "bus": return new Vector2(11f, 2.6f);
                case "truck": return new Vector2(8.5f, 2.5f);
                case "auto": return new Vector2(2.6f, 1.4f);
                case "two_wheeler": return new Vector2(1.9f, 0.7f);
                case "bicycle": return new Vector2(1.7f, 0.6f);
                case "pedestrian": return new Vector2(0.6f, 0.6f);
                case "cattle": return new Vector2(2.2f, 0.9f);
                case "pushcart": return new Vector2(2f, 1.2f);
                case "static": return new Vector2(3f, 1.6f);
                default: return new Vector2(4.2f, 1.8f);
            }
        }

        // ---- time lookup -----------------------------------------------------

        /// Index of the last frame at or before t, and the fraction to the next.
        public int FrameAt(float t, out float u)
        {
            u = 0f;
            int n = frames.Count;
            if (n == 0) return -1;
            if (t <= frames[0].t) return 0;
            if (t >= frames[n - 1].t) return n - 1;
            int lo = 0, hi = n - 1;
            while (hi - lo > 1)
            {
                int m = (lo + hi) / 2;
                if (frames[m].t <= t) lo = m; else hi = m;
            }
            float span = frames[hi].t - frames[lo].t;
            u = span > 1e-6f ? (t - frames[lo].t) / span : 0f;
            return lo;
        }

        /// Series index and fraction for time t.
        public int SeriesAt(float t, out float u)
        {
            u = 0f;
            int n = sT.Length;
            if (n == 0) return -1;
            if (t <= sT[0]) return 0;
            if (t >= sT[n - 1]) return n - 1;
            float k = (t - sT[0]) / Mathf.Max(dt, 1e-6f);
            int i = Mathf.Clamp(Mathf.FloorToInt(k), 0, n - 2);
            u = Mathf.Clamp01(k - i);
            return i;
        }

        public float Series(float[] a, float t)
        {
            if (a == null || a.Length == 0) return 0f;
            int i = SeriesAt(t, out float u);
            if (i >= a.Length - 1) return a[a.Length - 1];
            return Mathf.Lerp(a[i], a[i + 1], u);
        }

        /// Ego pose at t, interpolated from the 20 Hz series.
        public void EgoAt(float t, out float x, out float y, out float psi, out float v)
        {
            int i = SeriesAt(t, out float u);
            int j = Mathf.Min(i + 1, sT.Length - 1);
            x = Mathf.Lerp(sX[i], sX[j], u);
            y = Mathf.Lerp(sY[i], sY[j], u);
            psi = Frames.LerpAngle(sPsi[i], sPsi[j], u);
            v = Mathf.Lerp(sV[i], sV[j], u);
        }

        public float LatencyAt(float t)
        {
            float v = 0f;
            for (int i = 0; i < latT.Length && latT[i] <= t + 1e-4f; i++) v = latMs[i];
            return v;
        }
    }
}
