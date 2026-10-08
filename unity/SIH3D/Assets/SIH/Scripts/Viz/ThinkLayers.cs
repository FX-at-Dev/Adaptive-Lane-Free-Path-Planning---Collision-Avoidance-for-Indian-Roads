// What the car thinks, drawn into the scene.
//
// Every layer here is drawn from the planner's own snapshot (the THINK
// record of the latest replan at or before the playback time), never
// interpolated or invented: when the planner held a belief for 0.1 s, the
// overlay holds it for 0.1 s too. Layer numbers match the keys 1-9.
//
// Each layer is its own child object with a LineBatch, so its material, glow
// and draw order are on its MeshRenderer. Colours, widths and distances are
// fields below; change them in the inspector and the preview redraws.

using System.Collections.Generic;
using UnityEngine;

namespace Sih
{
    public enum Layer
    {
        Corridor = 0, Lidar = 1, Detections = 2, Tracks = 3, Predictions = 4,
        Occupancy = 5, Candidates = 6, Path = 7, Lookahead = 8,
    }

    /// Which of the nine layers are on.
    [System.Serializable]
    public class LayerSet
    {
        public const int Count = 9;
        public static readonly string[] Names =
        {
            "corridor", "lidar", "detections", "tracks", "predictions",
            "occupancy", "candidates", "selected path", "lookahead",
        };

        public bool corridor = true, lidar = true, detections = true, tracks = true, predictions = true;
        public bool occupancy = false, candidates = true, path = true, lookahead = true;

        public bool this[int i]
        {
            get
            {
                switch (i)
                {
                    case 0: return corridor; case 1: return lidar; case 2: return detections;
                    case 3: return tracks; case 4: return predictions; case 5: return occupancy;
                    case 6: return candidates; case 7: return path; case 8: return lookahead;
                }
                return false;
            }
            set
            {
                switch (i)
                {
                    case 0: corridor = value; break; case 1: lidar = value; break; case 2: detections = value; break;
                    case 3: tracks = value; break; case 4: predictions = value; break; case 5: occupancy = value; break;
                    case 6: candidates = value; break; case 7: path = value; break; case 8: lookahead = value; break;
                }
            }
        }

        public bool this[Layer l] { get => this[(int)l]; set => this[(int)l] = value; }

        public int Mask()
        {
            int m = 0;
            for (int i = 0; i < Count; i++) if (this[i]) m |= 1 << i;
            return m;
        }
    }

    [System.Serializable]
    public class ClassHeight
    {
        public string agentClass;
        public float height;
        public ClassHeight(string c, float h) { agentClass = c; height = h; }
    }

    [System.Serializable]
    public class ClassName
    {
        public string agentClass;
        public string label;
        public ClassName(string c, string l) { agentClass = c; label = l; }
    }

    [ExecuteAlways]
    public class ThinkLayers : MonoBehaviour
    {
        [Header("Layer objects")]
        public LineBatch corridor;
        public LineBatch detections;
        public LineBatch tracks;
        [Tooltip("Track velocity arrows and covariance ellipses, flat on the ground.")]
        public LineBatch trackGround;
        public LineBatch predictions;
        public LineBatch occupancy;
        public LineBatch candidates;
        public LineBatch path;
        public LineBatch lookahead;

        [Header("Freshness")]
        [Tooltip("Overlays from a planner snapshot older than this, seconds, are cleared rather than shown.")]
        public float staleAfter = 0.25f;

        [Header("Track labels")]
        [Tooltip("Pool of labels, one per track shown. Tracks beyond the pool get no label.")]
        public TextMesh[] labels = new TextMesh[0];
        public float labelHeightAboveBox = 0.55f;
        [Tooltip("Character size per metre of camera distance, so labels keep a steady size on screen.")]
        public float labelSizePerMetre = 0.0026f;
        public float labelMinSize = 0.02f, labelMaxSize = 0.25f;
        [Tooltip("Hide labels closer than this to the camera.")]
        public float labelHideCloserThan = 6f;
        [Tooltip("Added after the class name of a tentative / coasting track.")]
        public string tentativeTag = " ?", coastingTag = " ~";
        [Tooltip("Label tag of an object the car is slowing for until it is confirmed (it may be a ghost).")]
        public string unconfirmedTag = " (checking)";
        [Tooltip("Label text: {0} id, {1} class name, {2} tag, {3} speed in m/s. \\n is a line break.")]
        public string labelFormat = "#{0} {1}{2}\\n{3:0.0} m/s";
        [Tooltip("Name shown on the label for each class.")]
        public ClassName[] classNames =
        {
            new ClassName("car", "car"), new ClassName("bus", "bus"), new ClassName("truck", "truck"),
            new ClassName("auto", "auto-rickshaw"), new ClassName("two_wheeler", "two-wheeler"),
            new ClassName("bicycle", "bicycle"), new ClassName("pedestrian", "pedestrian"),
            new ClassName("cattle", "cattle"), new ClassName("pushcart", "pushcart"), new ClassName("static", "obstacle"),
        };

        [Header("1 Corridor")]
        public Color corridorColor = Palette.Plan;
        [Range(0, 1)] public float corridorAlpha = 0.55f;
        public float corridorWidth = 0.14f;
        public float corridorBehind = 8f, corridorAhead = 55f, corridorFade = 8f;
        public float corridorHeight = 0.04f;

        [Header("3 Detections (camera, radar, lidar)")]
        public Color[] sensorColors = { Palette.Hex("#facc15"), Palette.Hex("#e879f9"), Palette.Hex("#22d3ee") };
        public float detectionHeight = 2.3f;
        [Tooltip("Extra height per sensor, so co-located hits stay legible.")]
        public float detectionHeightStep = 0.35f;
        public float detectionStemWidth = 0.045f;
        public float detectionDiamond = 0.18f;
        public float detectionGroundDot = 0.45f;
        [Tooltip("Draw the box a LiDAR saw, flat on the ground, where the detection has one.")]
        public bool drawDetectionBoxes = true;
        public float detectionBoxLineWidth = 0.05f;
        public float detectionBoxHeight = 0.06f;
        [Range(0, 1)] public float detectionGroundDotAlpha = 0.55f;
        [Tooltip("Height of the stem's foot and the ground dot.")]
        public float detectionBaseHeight = 0.05f;
        [Range(0, 1)] public float stemBottomAlpha = 0f, stemTopAlpha = 0.9f;
        public float diamondLineWidth = 0.05f;

        [Header("4 Tracks")]
        public Color confirmedColor = Palette.Track;
        public Color tentativeColor = Palette.A(Palette.Dim, 0.6f);
        public Color coastingColor = Palette.A(Palette.Hex("#93c5fd"), 0.6f);
        [Tooltip("An object the car slows for but does not stop for until it is confirmed: it may be a ghost.")]
        public Color unconfirmedColor = Palette.A(Palette.Hex("#fcd34d"), 0.55f);
        public Color leadColor = Palette.Hex("#fbbf24");
        public Color crossingColor = Palette.Hex("#fb923c");
        [Tooltip("A track whose footprint is in the ego's path (measured along the road).")]
        public Color inPathColor = Palette.Hex("#f87171");
        [Tooltip("Added to the label of a stationary track.")]
        public string stillTag = " (still)";
        public float boxPadding = 0.3f;
        [Tooltip("Added to the class height for the box.")]
        public float boxHeightPadding = 0.15f;
        [Tooltip("Height of the box's bottom edges above the ground.")]
        public float boxBottomHeight = 0.03f;
        [Range(0, 1)] public float boxVerticalEdgeAlpha = 0.25f;
        public float confirmedLineWidth = 0.06f, otherLineWidth = 0.035f;
        [Tooltip("Class height used when a class is not in the list below.")]
        public float defaultClassHeight = 1.5f;

        [Tooltip("No arrow below this speed, m/s.")]
        public float arrowMinSpeed = 0.3f;
        [Tooltip("Arrow length in seconds of travel.")]
        public float arrowSeconds = 1f;
        public float arrowWidth = 0.12f;
        public float arrowHeadLength = 0.5f, arrowHeadHalfWidth = 0.3f;
        public float arrowHeight = 0.08f;
        [Range(0, 1)] public float arrowAlpha = 0.9f;

        [Tooltip("Ellipse radius in standard deviations.")]
        public float ellipseSigma = 3f;
        public float ellipseWidth = 0.06f;
        public float ellipseHeight = 0.06f;
        [Range(0, 1)] public float ellipseAlpha = 0.75f;
        [Tooltip("No ellipse when it is smaller than this, metres.")]
        public float ellipseMinRadius = 0.05f;
        [Range(6, 128)] public int ellipseSegments = 28;
        public ClassHeight[] classHeights =
        {
            new ClassHeight("car", 1.5f), new ClassHeight("bus", 3.2f), new ClassHeight("truck", 3f),
            new ClassHeight("auto", 1.85f), new ClassHeight("two_wheeler", 1.55f), new ClassHeight("bicycle", 1.65f),
            new ClassHeight("pedestrian", 1.7f), new ClassHeight("cattle", 1.45f), new ClassHeight("pushcart", 1.3f),
            new ClassHeight("static", 1.3f),
        };

        [Header("5 Predictions (keep, brake, left, right)")]
        public Color[] modeColors = { Palette.Hex("#60a5fa"), Palette.Hex("#fbbf24"), Palette.Hex("#f472b6"), Palette.Hex("#a78bfa") };
        public float predictionWidth = 0.16f, predictionWidthPerWeight = 0.2f;
        public float predictionDot = 0.22f;
        [Range(3, 64)] public int predictionDotSegments = 10;
        public float predictionHeight = 0.12f;
        [Range(0, 1)] public float predictionAlpha = 0.25f, predictionAlphaPerWeight = 0.75f;
        [Tooltip("How much the ribbon fades towards its far end.")]
        [Range(0, 1)] public float predictionFade = 0.7f;

        [Header("6 Occupancy")]
        public Color occupancyColor = Palette.Hex("#f87171");
        public float occupancyAlpha = 0.05f, occupancyAlphaPerWeight = 0.22f;
        [Range(0, 1)] public float occupancyFade = 0.6f;

        [Header("7 Candidates")]
        public Color bestCandidate = Palette.Hex("#5eead4");
        public Color worstCandidate = Palette.Hex("#facc15");
        [Tooltip("By status code: ok, risk, conflict, slope, curvature, speed, lat. accel.")]
        public Color[] rejectedColors =
        {
            Palette.Hex("#22d3aa"), Palette.Hex("#f87171"), Palette.Hex("#ef4444"), Palette.Hex("#64748b"),
            Palette.Hex("#94a3b8"), Palette.Hex("#a78bfa"), Palette.Hex("#c084fc"),
        };
        public float candidateWidth = 0.05f, rejectedWidth = 0.035f;
        public float candidateHeight = 0.2f;
        [Range(0, 1)] public float bestCandidateAlpha = 0.85f, worstCandidateAlpha = 0.4f;
        [Tooltip("Below 1 spreads the colours of the cheaper candidates apart.")]
        public float costColorCurve = 0.5f;
        [Tooltip("Rejection codes up to this one (risk, conflict) are drawn stronger.")]
        public int strongRejectUpToCode = 2;
        [Range(0, 1)] public float strongRejectAlpha = 0.6f, weakRejectAlpha = 0.32f;

        [Header("8 Selected path")]
        public Color pathColor = Palette.Plan;
        public float pathWidth = 0.42f;
        public float pathHeight = 0.07f;
        [Range(0, 1)] public float pathAlpha = 0.95f, pathFade = 0.55f;
        [Range(0, 1)] public float footprintAlpha = 0.07f, footprintFade = 0.6f;
        [Tooltip("0: the ego's width from the run.")]
        public float footprintWidth = 0f;

        [Header("9 Lookahead")]
        public Color lookColor = Palette.Look;
        public float lookDot = 0.4f, lookStem = 1.2f;
        public float lookHeight = 0.09f;
        public float lookStemWidth = 0.06f, lookLinkWidth = 0.05f;
        [Range(0, 1)] public float lookLinkAlpha = 0.6f;

        RunData run;
        float[] arc;
        int shownFrame = -2, shownMask = -1;

        public void Bind(RunData r)
        {
            run = r;
            int n = r.roadX.Length;
            arc = new float[n];
            for (int i = 1; i < n; i++)
                arc[i] = arc[i - 1] + Vector2.Distance(new Vector2(r.roadX[i], r.roadY[i]), new Vector2(r.roadX[i - 1], r.roadY[i - 1]));
            Invalidate();
        }

        public void Invalidate() => shownFrame = -2;

        /// Redraw if the planner snapshot or the layer mask changed.
        public void Apply(float t, LayerSet on)
        {
            if (run == null) return;
            int fi = run.FrameAt(t, out _);
            // Nothing older than Stale After is shown: a stalled or restarted
            // run must not leave last moment's boxes standing in the world.
            if (fi >= 0 && t - run.frames[fi].t > staleAfter) fi = -1;
            int mask = on.Mask();
            if (fi == shownFrame && mask == shownMask) return;
            shownFrame = fi;
            shownMask = mask;
            Rebuild(fi < 0 ? null : run.frames[fi], on);
        }

        /// Labels face the camera at a steady on-screen size.
        public void FaceLabels(Camera cam)
        {
            if (cam == null) return;
            var ct = cam.transform;
            foreach (var l in labels)
            {
                if (l == null || !l.gameObject.activeSelf) continue;
                l.transform.rotation = ct.rotation;
                float d = Vector3.Distance(ct.position, l.transform.position);
                l.characterSize = Mathf.Clamp(labelSizePerMetre * d, labelMinSize, labelMaxSize);
                var mr = l.GetComponent<MeshRenderer>();
                if (mr != null) mr.enabled = d > labelHideCloserThan;
            }
        }

        LineBatch[] All => new[] { corridor, detections, tracks, trackGround, predictions, occupancy, candidates, path, lookahead };

        void Rebuild(Frame f, LayerSet on)
        {
            foreach (var b in All) if (b != null) b.Clear();
            int nl = 0;

            if (f != null)
            {
                var k = f.think;
                if (k != null)
                {
                    if (on.corridor && corridor) Corridor(f, k);
                    if (on.detections && detections) Detections(k);
                    if (on.tracks && tracks) nl = Tracks(k);
                    if (on.predictions || on.occupancy) Predictions(k, on);
                    if (on.candidates && candidates) Candidates(k);
                    if (on.lookahead && lookahead) Lookahead(f, k);
                }
                if (on.path && path) Path(f);
            }

            Show(corridor, on.corridor);
            Show(detections, on.detections);
            Show(tracks, on.tracks);
            Show(trackGround, on.tracks);
            Show(predictions, on.predictions);
            Show(occupancy, on.occupancy);
            Show(candidates, on.candidates);
            Show(path, on.path);
            Show(lookahead, on.lookahead);
            for (int i = nl; i < labels.Length; i++)
                if (labels[i] != null && labels[i].gameObject.activeSelf) labels[i].gameObject.SetActive(false);
        }

        static void Show(LineBatch b, bool on)
        {
            if (b == null) return;
            b.Flush();
            b.SetVisible(on);
        }

        // ---- 1 corridor ------------------------------------------------------

        void Corridor(Frame f, Think k)
        {
            float dl = k.Info(8);
            if (dl <= 0f) return;
            int n = run.roadX.Length;
            int near = Nearest(f.ex, f.ey);
            float s0 = arc[near] - corridorBehind, s1 = arc[near] + corridorAhead;
            var L = new List<Vector3>();
            var R = new List<Vector3>();
            var cs = new List<Color>();
            for (int i = 0; i < n; i++)
            {
                if (arc[i] < s0 || arc[i] > s1) continue;
                float psi = run.roadPsi[i];
                float nx = -Mathf.Sin(psi), ny = Mathf.Cos(psi);
                L.Add(Frames.Pos(run.roadX[i] + nx * dl, run.roadY[i] + ny * dl, corridorHeight));
                R.Add(Frames.Pos(run.roadX[i] - nx * dl, run.roadY[i] - ny * dl, corridorHeight));
                float fade = Mathf.Clamp01(Mathf.Min(arc[i] - s0, s1 - arc[i]) / Mathf.Max(corridorFade, 1e-3f));
                cs.Add(Palette.A(corridorColor, corridorAlpha * fade));
            }
            corridor.Strip(L, corridorWidth, Color.white, true, false, cs);
            corridor.Strip(R, corridorWidth, Color.white, true, false, cs);
        }

        int Nearest(float x, float y)
        {
            int best = 0; float bd = float.MaxValue;
            for (int i = 0; i < run.roadX.Length; i++)
            {
                float dx = run.roadX[i] - x, dy = run.roadY[i] - y;
                float d = dx * dx + dy * dy;
                if (d < bd) { bd = d; best = i; }
            }
            return best;
        }

        // ---- 3 detections ----------------------------------------------------

        void Detections(Think k)
        {
            var b = detections;
            foreach (var d in k.dets)
            {
                float x = d.x, y = d.y;
                int s = d.sensor;
                var c = s < sensorColors.Length ? sensorColors[s] : Color.white;
                // The box the sensor saw (a LiDAR's visible faces), in the world.
                if (drawDetectionBoxes && d.HasBox)
                    Rect(b, x, y, d.theta, d.L, d.W, detectionBoxHeight, detectionBoxLineWidth, Palette.A(c, 0.9f));
                float h = detectionHeight + detectionHeightStep * s;
                var p = Frames.Pos(x, y);
                b.Seg(p + Vector3.up * detectionBaseHeight, p + Vector3.up * h, detectionStemWidth,
                      Palette.A(c, stemBottomAlpha), Palette.A(c, stemTopAlpha));
                Diamond(b, p + Vector3.up * h, detectionDiamond, diamondLineWidth, c);
                b.Dot(p + Vector3.up * detectionBaseHeight, detectionGroundDot, Palette.A(c, detectionGroundDotAlpha));
            }
        }

        static void Diamond(LineBatch b, Vector3 c, float r, float w, Color col)
        {
            var p = new[] { c + Vector3.up * r, c + Vector3.right * r, c - Vector3.up * r, c - Vector3.right * r };
            b.Strip(p, w, col, false, true);
        }

        // ---- 4 tracks --------------------------------------------------------

        float HeightOf(string cls)
        {
            foreach (var h in classHeights) if (h != null && h.agentClass == cls) return h.height;
            return defaultClassHeight;
        }

        int Tracks(Think k)
        {
            int lead = Mathf.RoundToInt(k.Ra(1)), cross = Mathf.RoundToInt(k.Ra(6));
            int nl = 0;
            foreach (var tr in k.tracks)
            {
                int id = tr.id, st = tr.status;
                float x = tr.x, y = tr.y, psi = tr.psi, v = tr.still ? 0f : tr.v;
                string cls = tr.cls;

                Color c = confirmedColor;
                if (st == 0) c = tentativeColor;
                else if (st == 2) c = coastingColor;
                else if (st == 3) c = unconfirmedColor;
                if (tr.inPath && st != 0 && st != 3) c = inPathColor;
                if (id == cross && cross > 0) c = crossingColor;
                else if (id == lead && lead > 0) c = leadColor;

                // The footprint the stack decided with: its centre, its long
                // axis and its size, in the world. Older runs carry no
                // footprint; then the class size along the motion heading.
                float L, W, theta;
                if (tr.HasFootprint) { L = tr.L; W = tr.W; theta = tr.theta; }
                else { var size = run.SizeOf(cls); L = size.x; W = size.y; theta = psi; }
                float h = HeightOf(cls) + boxHeightPadding;
                WireBox(x, y, theta, L + boxPadding, W + boxPadding, h, c, st == 1 ? confirmedLineWidth : otherLineWidth,
                        !tr.still && v > arrowMinSpeed);

                // Velocity arrow on the ground: arrowSeconds of travel.
                if (v > arrowMinSpeed && st != 0 && trackGround != null)
                {
                    float len = v * arrowSeconds;
                    var a = Frames.Pos(x, y, arrowHeight);
                    var e = Frames.Pos(x + Mathf.Cos(psi) * len, y + Mathf.Sin(psi) * len, arrowHeight);
                    var ac = Palette.A(c, arrowAlpha);
                    trackGround.Seg(a, e, arrowWidth, ac, true);
                    float lb = len - arrowHeadLength;
                    var back = Frames.Pos(x + Mathf.Cos(psi) * lb, y + Mathf.Sin(psi) * lb, arrowHeight);
                    var sideU = Frames.Pos(-Mathf.Sin(psi), Mathf.Cos(psi)) * arrowHeadHalfWidth;
                    trackGround.Seg(e, back + sideU, arrowWidth, ac, true);
                    trackGround.Seg(e, back - sideU, arrowWidth, ac, true);
                }

                if (trackGround != null) Ellipse(x, y, tr.pxx, tr.pxy, tr.pyy, Palette.A(c, ellipseAlpha));

                if (nl < labels.Length && labels[nl] != null)
                {
                    var lab = labels[nl++];
                    if (!lab.gameObject.activeSelf) lab.gameObject.SetActive(true);
                    lab.transform.position = Frames.Pos(x, y, h + labelHeightAboveBox);
                    string tag = (st == 0 ? tentativeTag : st == 2 ? coastingTag : st == 3 ? unconfirmedTag : "") + (tr.still ? stillTag : "");
                    lab.text = LabelText(id, ClassLabel(cls), tag, v);
                    lab.color = c;
                }
            }
            return nl;
        }

        string ClassLabel(string cls)
        {
            foreach (var n in classNames) if (n != null && n.agentClass == cls) return n.label;
            return cls;
        }

        string LabelText(int id, string cls, string tag, float v)
        {
            // Typed in the inspector, a line break is written as \n.
            try { return string.Format((labelFormat ?? "").Replace("\\n", "\n"), id, cls, tag, v); }
            catch (System.FormatException) { return $"#{id} {cls}{tag}"; }
        }

        /// A flat oriented rectangle at height h.
        static void Rect(LineBatch b, float x, float y, float th, float L, float W, float h, float w, Color c)
        {
            float cp = Mathf.Cos(th), sp = Mathf.Sin(th);
            var p = new Vector3[4];
            float[] lx = { L / 2, L / 2, -L / 2, -L / 2 }, ly = { W / 2, -W / 2, -W / 2, W / 2 };
            for (int i = 0; i < 4; i++) p[i] = Frames.Pos(x + cp * lx[i] - sp * ly[i], y + sp * lx[i] + cp * ly[i], h);
            b.Strip(p, w, c, true, true);
        }

        /// The footprint box: L along the axis psi, W across it, H tall. The
        /// heading tick on the roof only for a moving object (a parked one has
        /// an axis, not a front).
        void WireBox(float x, float y, float psi, float L, float W, float H, Color c, float w, bool tick = true)
        {
            var b = tracks;
            float cp = Mathf.Cos(psi), sp = Mathf.Sin(psi);
            var g = new Vector3[4];
            float[] lx = { L / 2, L / 2, -L / 2, -L / 2 }, ly = { W / 2, -W / 2, -W / 2, W / 2 };
            for (int i = 0; i < 4; i++)
                g[i] = Frames.Pos(x + cp * lx[i] - sp * ly[i], y + sp * lx[i] + cp * ly[i]);
            var up = Vector3.up * H;
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                b.Seg(g[i] + Vector3.up * boxBottomHeight, g[j] + Vector3.up * boxBottomHeight, w, c);
                b.Seg(g[i] + up, g[j] + up, w, c);
                b.Seg(g[i], g[i] + up, w, Palette.A(c, c.a * boxVerticalEdgeAlpha), c);
            }
            // Heading tick on the roof.
            if (!tick) return;
            var front = Frames.Pos(x + cp * L / 2, y + sp * L / 2, H);
            b.Seg(Frames.Pos(x, y, H), front, w, c);
        }

        void Ellipse(float x, float y, float a, float bxy, float c, Color col)
        {
            float tr = (a + c) / 2f, df = (a - c) / 2f;
            float rt = Mathf.Sqrt(df * df + bxy * bxy);
            float l1 = Mathf.Max(tr + rt, 1e-6f), l2 = Mathf.Max(tr - rt, 1e-6f);
            float th = 0.5f * Mathf.Atan2(2f * bxy, a - c);
            float r1 = ellipseSigma * Mathf.Sqrt(l1), r2 = ellipseSigma * Mathf.Sqrt(l2);
            if (r1 < ellipseMinRadius) return;
            int seg = Mathf.Max(6, ellipseSegments);
            var p = new Vector3[seg];
            float ct = Mathf.Cos(th), st = Mathf.Sin(th);
            for (int i = 0; i < seg; i++)
            {
                float u = i * Mathf.PI * 2f / seg;
                float ex = r1 * Mathf.Cos(u), ey = r2 * Mathf.Sin(u);
                p[i] = Frames.Pos(x + ct * ex - st * ey, y + st * ex + ct * ey, ellipseHeight);
            }
            trackGround.Strip(p, ellipseWidth, col, true, true);
        }

        // ---- 5/6 predictions and swept occupancy -----------------------------

        void Predictions(Think k, LayerSet on)
        {
            bool doPred = on.predictions && predictions != null;
            bool doOcc = on.occupancy && occupancy != null;
            if (!doPred && !doOcc) return;
            int np = k.predPts;
            if (np < 2) return;
            var pts = new Vector3[np];
            var cs = new Color[np];
            for (int h = 0; h < k.NPred; h++)
            {
                int mode = Mathf.Clamp(Mathf.RoundToInt(k.pred[4 * h + 1]), 0, 3);
                float w = k.pred[4 * h + 2], r = k.pred[4 * h + 3];
                int o = 2 * np * h;
                if (o + 2 * np > k.predXY.Length) break;
                for (int i = 0; i < np; i++)
                    pts[i] = Frames.Pos(k.predXY[o + 2 * i], k.predXY[o + 2 * i + 1], predictionHeight);

                if (doOcc)
                {
                    for (int i = 0; i < np; i++)
                        cs[i] = Palette.A(occupancyColor, (occupancyAlpha + occupancyAlphaPerWeight * w) * (1f - occupancyFade * i / (np - 1f)));
                    occupancy.Strip(pts, 2f * r, Color.white, true, false, cs);
                }
                if (doPred)
                {
                    var c = mode < modeColors.Length ? modeColors[mode] : Color.white;
                    for (int i = 0; i < np; i++)
                        cs[i] = Palette.A(c, (predictionAlpha + predictionAlphaPerWeight * w) * (1f - predictionFade * i / (np - 1f)));
                    predictions.Strip(pts, predictionWidth + predictionWidthPerWeight * w, Color.white, true, false, cs);
                    // A dot every step: spacing shows the predicted speed.
                    for (int i = 1; i < np; i++) predictions.Dot(pts[i] + Vector3.up * 0.01f, predictionDot, cs[i], Mathf.Max(3, predictionDotSegments));
                }
            }
        }

        // ---- 7 candidates ----------------------------------------------------

        void Candidates(Think k)
        {
            int np = k.candPts;
            if (np < 2) return;
            float lo = float.MaxValue, hi = float.MinValue;
            for (int i = 0; i < k.NCand; i++)
            {
                if (Mathf.RoundToInt(k.cand[3 * i]) != 0) continue;
                float cost = k.cand[3 * i + 1];
                lo = Mathf.Min(lo, cost); hi = Mathf.Max(hi, cost);
            }
            var pts = new Vector3[np];
            // Rejected first, so the feasible fan draws over them.
            for (int pass = 0; pass < 2; pass++)
                for (int i = 0; i < k.NCand; i++)
                {
                    int code = Mathf.Clamp(Mathf.RoundToInt(k.cand[3 * i]), 0, rejectedColors.Length - 1);
                    if ((code == 0) != (pass == 1)) continue;
                    int o = 2 * np * i;
                    if (o + 2 * np > k.candXY.Length) break;
                    for (int j = 0; j < np; j++)
                        pts[j] = Frames.Pos(k.candXY[o + 2 * j], k.candXY[o + 2 * j + 1], candidateHeight);
                    Color c;
                    float w;
                    if (code == 0)
                    {
                        float u = hi > lo ? (k.cand[3 * i + 1] - lo) / (hi - lo) : 0f;
                        c = Color.Lerp(bestCandidate, worstCandidate, Mathf.Pow(u, Mathf.Max(costColorCurve, 0.01f)));
                        c.a = Mathf.Lerp(bestCandidateAlpha, worstCandidateAlpha, u);
                        w = candidateWidth;
                    }
                    else
                    {
                        c = Palette.A(rejectedColors[code], code <= strongRejectUpToCode ? strongRejectAlpha : weakRejectAlpha);
                        w = rejectedWidth;
                    }
                    candidates.Strip(pts, w, c);
                }
        }

        // ---- 8 selected path, 9 lookahead ------------------------------------

        void Path(Frame f)
        {
            int n = Mathf.Min(f.trajX.Length, f.trajY.Length);
            if (n < 2) return;
            var p = new Vector3[n];
            var cs = new Color[n];
            for (int i = 0; i < n; i++)
            {
                p[i] = Frames.Pos(f.trajX[i], f.trajY[i], pathHeight);
                cs[i] = Palette.A(pathColor, pathAlpha * (1f - pathFade * i / (n - 1f)));
            }
            path.Strip(p, pathWidth, Color.white, true, false, cs);
            // The ego's own footprint swept along it, faintly.
            for (int i = 0; i < n; i++) cs[i] = Palette.A(pathColor, footprintAlpha * (1f - footprintFade * i / (n - 1f)));
            path.Strip(p, footprintWidth > 0f ? footprintWidth : run.egoW, Color.white, true, false, cs);
        }

        void Lookahead(Frame f, Think k)
        {
            var p = Frames.Pos(k.look.x, k.look.y, lookHeight);
            lookahead.Dot(p, lookDot, lookColor);
            lookahead.Seg(p, p + Vector3.up * lookStem, lookStemWidth, lookColor);
            var e = Frames.Pos(f.ex, f.ey, lookHeight);
            lookahead.Seg(e, p, lookLinkWidth, Palette.A(lookColor, lookLinkAlpha), true);
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            Invalidate();
            if (!Application.isPlaying) SihReplay.RequestEditorRefresh();
        }
#endif
    }
}
