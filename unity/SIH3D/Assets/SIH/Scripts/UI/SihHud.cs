// The heads-up display. The panels, texts, colours and layout all live in the
// scene's HUD canvas and can be edited there; this component only writes the
// numbers into them.
//
// Every number is read from the run: the 20 Hz series for what the car did,
// the latest THINK snapshot for what the planner believed. Where those differ
// (truth vs perceived clearance) both are shown, side by side.

using UnityEngine;
using UnityEngine.UI;

namespace Sih
{
    [ExecuteAlways]
    public class SihHud : MonoBehaviour
    {
        [System.Serializable]
        public class MetricView
        {
            public Text label, value, sub;
        }

        [System.Serializable]
        public class StateColor
        {
            public string state;
            public Color color;
            public StateColor(string s, Color c) { state = s; color = c; }
        }

        [Header("Groups")]
        [Tooltip("Hidden by H. The footer (timeline) stays.")]
        public GameObject hudRoot;
        [Tooltip("Shown by F1.")]
        public GameObject helpPanel;

        [Header("Scenario panel")]
        public Text scenarioTitle;
        public Text scenarioDesc;
        public Text statusLine;
        [Tooltip("Shown when playing a recorded run.")]
        public GameObject replayBadge;
        [Tooltip("Shown when the stack is driving live.")]
        public GameObject liveBadge;
        [Tooltip("Live: what the Live Link is doing (connecting, a judge drop, the goal).")]
        public Text liveStatus;

        [Header("State panel")]
        public Image stateChip;
        public Text stateText;
        public Text vCapText, dMaxText, riskTolText;
        public Text reasonText;

        [Header("Metrics")]
        public MetricView speed = new MetricView();
        public MetricView ttc = new MetricView();
        public MetricView tracks = new MetricView();
        public MetricView clearance = new MetricView();
        public MetricView risk = new MetricView();
        public MetricView latency = new MetricView();

        [Header("Planner panel")]
        public Text plannerSummary;
        public Text plannerLegend;
        public RectTransform acceptSegment;
        [Tooltip("One per rejection reason, in the order of Reject Names.")]
        public RectTransform[] rejectSegments = new RectTransform[6];

        [Header("Layers and timeline")]
        public SihLayerRow[] layerRows = new SihLayerRow[9];
        public SihTimeline timeline;

        [Header("Colours")]
        public Color valueColor = Palette.Ink;
        public Color tracksColor = Palette.Track;
        public Color warnColor = Palette.Hex("#fbbf24");
        public Color cautionColor = Palette.Hex("#fb923c");
        public Color dangerColor = Palette.Hex("#f87171");
        public Color goodColor = Palette.Goal;
        public Color highlightColor = Palette.Ink;
        public StateColor[] stateColors =
        {
            new StateColor("CRUISE", Palette.State["CRUISE"]),
            new StateColor("FOLLOW", Palette.State["FOLLOW"]),
            new StateColor("NUDGE", Palette.State["NUDGE"]),
            new StateColor("YIELD", Palette.State["YIELD"]),
            new StateColor("CREEP", Palette.State["CREEP"]),
            new StateColor("STOP", Palette.State["STOP"]),
        };
        public Color unknownStateColor = Palette.Dim;

        [Header("Thresholds")]
        [Tooltip("TTC below this, seconds, is shown in the warn colour.")]
        public float ttcWarn = 3f;
        [Tooltip("Clearance below this, metres, is danger; below the second, caution.")]
        public float clearanceDanger = 0.5f, clearanceCaution = 1f;
        [Tooltip("Plan risk above this fraction of the tolerance is shown in the caution colour.")]
        public float riskCautionFraction = 0.7f;

        [Header("Text")]
        [Tooltip("Font size of the km/h after the speed.")]
        public int speedUnitSize = 14;
        public string emptyValue = "—";
        public string noCrossingText = "no crossing conflict";
        public string noSnapshotText = "no planner snapshot";
        public string noRejectionsText = "no rejections";
        [Tooltip("After the latency budget.")]
        public string latencyBudgetNote = "(Octave)";
        public string playingText = "playing", pausedText = "paused", editPreviewText = "edit preview";
        [Tooltip("Live status line: {0} time, {1} real-time factor, {2} stack ms per step, {3} playing/paused, {4} camera, {5} fps.")]
        public string liveStatusFormat = "t {0:0.0} s    {1:0.00}× real time    stack {2:0} ms/step    {3}    cam {4}    {5:0} fps";
        [Tooltip("Real-time factor below this is shown in the warn colour.")]
        public float rtfWarn = 0.95f;

        [Header("Rejections")]
        public string[] rejectNames = { "lon", "slope", "curv", "speed", "a_lat", "risk" };
        public Color[] rejectColors =
        {
            Palette.Hex("#475569"), Palette.CandReject[3], Palette.CandReject[4],
            Palette.CandReject[5], Palette.CandReject[6], Palette.CandReject[1],
        };

        public bool HelpOpen => helpPanel != null && helpPanel.activeSelf;

        public void ToggleVisible()
        {
            if (hudRoot != null) hudRoot.SetActive(!hudRoot.activeSelf);
        }

        public void ToggleHelp()
        {
            if (helpPanel != null) helpPanel.SetActive(!helpPanel.activeSelf);
        }

        public void Bind(SihReplay replay)
        {
            bool live = replay.IsLive;
            if (replayBadge != null && replayBadge.activeSelf == live) replayBadge.SetActive(!live);
            if (liveBadge != null && liveBadge.activeSelf != live) liveBadge.SetActive(live);
            if (liveStatus != null && liveStatus.gameObject.activeSelf != live) liveStatus.gameObject.SetActive(live);
            // The band is a picture of the recorded run; a live run has no
            // future to draw, so only the playhead shows.
            if (timeline != null && timeline.band != null) timeline.band.enabled = !live;
            var run = replay.Run;
            if (run == null) return;
            Set(scenarioTitle, Pretty(run.name));
            Set(scenarioDesc, run.desc);
            if (timeline != null) timeline.replay = replay;
            for (int i = 0; i < layerRows.Length; i++)
                if (layerRows[i] != null) layerRows[i].replay = replay;
        }

        public void Apply(SihReplay replay)
        {
            var link = replay.IsLive ? replay.liveLink : null;
            if (link != null)
            {
                Set(liveStatus, link.status);
                if (replay.Run == null || replay.Run.sT.Length == 0)
                {
                    Set(statusLine, link.status);
                    return;
                }
            }
            var run = replay.Run;
            if (run == null) return;
            float t = replay.t;
            int fi = run.FrameAt(t, out _);
            var f = fi >= 0 ? run.frames[fi] : null;
            var k = f?.think;

            string cam = replay.cameraRig != null ? CameraRig.ModeName[(int)replay.cameraRig.mode] : "-";
            string play = Application.isPlaying ? (replay.playing ? playingText : pausedText) : editPreviewText;
            if (link != null)
            {
                string line;
                try { line = string.Format(liveStatusFormat, t, link.realTimeFactor, link.StepTotalMs, play, cam, replay.fps); }
                catch (System.FormatException) { line = $"t {t:0.0} s"; }
                Set(statusLine, line);
                if (statusLine != null)
                {
                    var c = link.realTimeFactor < rtfWarn ? warnColor : valueColor;
                    if (statusLine.color != c) statusLine.color = c;
                }
            }
            else
                Set(statusLine, $"t {t:0.0} / {run.T1:0.0} s    {play} {replay.speed:0.##}×    cam {cam}    {replay.fps:0} fps");

            // State and reason.
            string state = f != null ? f.state : emptyValue;
            Set(stateText, state);
            if (stateChip != null) stateChip.color = OfState(state);
            Set(vCapText, k != null ? $"v_cap  {Bp(k, 0):0.0} m/s" : "");
            Set(dMaxText, k != null ? $"d_max  {Bp(k, 1):0.00} m" : "");
            Set(riskTolText, k != null ? $"risk tol {Bp(k, 2):0.00}" : "");
            Set(reasonText, k != null ? k.reason : "");

            // Metrics.
            float v = replay.EgoV, vcap = run.Series(run.sVCap, t);
            Metric(speed, $"{v * 3.6f:0} <size={speedUnitSize}>km/h</size>", $"{v:0.0} m/s · cap {vcap:0.0}", valueColor);

            float tt = k != null ? k.Ra(0) : 999f;
            float ttX = k != null ? k.Ra(5) : 999f;
            Metric(ttc, tt >= 998f ? emptyValue : $"{tt:0.0} s",
                   ttX >= 998f ? noCrossingText : $"crossing #{k.Ra(6):0} in {ttX:0.0} s",
                   tt < ttcWarn ? warnColor : valueColor);

            int conf = 0, tot = 0;
            if (k != null)
                foreach (var tr in k.tracks) { tot++; if (tr.status == 1) conf++; }
            int dets = k != null ? k.NDets : 0;
            Metric(tracks, $"{conf}", $"{tot} total · {dets} detections", tracksColor);

            float mcT = run.Series(run.sMinClear, t);
            float mcP = k != null ? k.Ra(4) : 999f;
            Metric(clearance, mcT >= 998f ? emptyValue : $"{mcT:0.00} m",
                   "truth · perceived " + (mcP >= 998f ? emptyValue : $"{mcP:0.00} m"),
                   mcT < clearanceDanger ? dangerColor : mcT < clearanceCaution ? cautionColor : goodColor);

            float rk = run.Series(run.sRisk, t);
            float tol = k != null ? Bp(k, 2) : 0f;
            Metric(risk, rk < 0 ? emptyValue : $"{rk:0.000}", tol > 0 ? $"tolerance {tol:0.00}" : "",
                   tol > 0 && rk > tol * riskCautionFraction ? cautionColor : valueColor);

            float lat = run.LatencyAt(t);
            Metric(latency, $"{lat:0} ms", $"budget {run.latencyBudget:0} ms {latencyBudgetNote}",
                   lat > run.latencyBudget ? warnColor : valueColor);

            Planner(k);

            for (int i = 0; i < layerRows.Length; i++)
                if (layerRows[i] != null) layerRows[i].Show(replay.layers[layerRows[i].layer]);

            if (timeline != null)
                timeline.Show(link != null ? (run.tEnd > 0 ? t / run.tEnd : 0f) : Mathf.InverseLerp(run.T0, run.T1, t));
        }

        void Planner(Think k)
        {
            if (k == null)
            {
                Set(plannerSummary, noSnapshotText);
                Set(plannerLegend, "");
                Segment(acceptSegment, 0f, 0f);
                foreach (var s in rejectSegments) Segment(s, 0f, 0f);
                return;
            }
            int ne = Mathf.RoundToInt(k.Info(0)), nf = Mathf.RoundToInt(k.Info(1));
            string hi = ColorUtility.ToHtmlStringRGB(highlightColor);
            Set(plannerSummary, $"<color=#{hi}><b>{nf}</b></color> feasible of {ne} · d_lim {k.Info(8):0.00} m");

            float rej = 0f;
            for (int i = 0; i < 6; i++) rej += Mathf.Max(0f, k.Info(2 + i));
            float accept = Mathf.Max(nf, 0);
            float all = rej + accept;
            float x = 0f;
            float w = all > 0 ? accept / all : 0f;
            Segment(acceptSegment, x, w); x += w;
            for (int i = 0; i < rejectSegments.Length; i++)
            {
                w = all > 0 && i < 6 ? Mathf.Max(0f, k.Info(2 + i)) / all : 0f;
                Segment(rejectSegments[i], x, w); x += w;
            }

            var legend = new System.Text.StringBuilder();
            for (int i = 0; i < 6 && i < rejectNames.Length; i++)
            {
                int n = Mathf.RoundToInt(k.Info(2 + i));
                if (n <= 0) continue;
                var c = i < rejectColors.Length ? rejectColors[i] : Color.grey;
                legend.Append($"<color=#{ColorUtility.ToHtmlStringRGB(c)}>■</color> {rejectNames[i]} {n}  ");
            }
            Set(plannerLegend, legend.Length > 0 ? legend.ToString() : noRejectionsText);
        }

        static void Segment(RectTransform r, float x, float w)
        {
            if (r == null) return;
            var a = new Vector2(x, 0f);
            var b = new Vector2(x + w, 1f);
            if (r.anchorMin != a) r.anchorMin = a;
            if (r.anchorMax != b) r.anchorMax = b;
            bool on = w > 1e-4f;
            if (r.gameObject.activeSelf != on) r.gameObject.SetActive(on);
        }

        void Metric(MetricView m, string value, string sub, Color c)
        {
            if (m == null) return;
            Set(m.value, value);
            Set(m.sub, sub);
            if (m.value != null && m.value.color != c) m.value.color = c;
        }

        public Color OfState(string s)
        {
            foreach (var sc in stateColors)
                if (sc != null && sc.state == s) return sc.color;
            // A state newer than the list saved in the scene (REVERSE).
            if (s != null && Palette.State.TryGetValue(s, out var pc)) return pc;
            return unknownStateColor;
        }

        static float Bp(Think k, int i) => k.bp != null && k.bp.Length > i ? k.bp[i] : 0f;

        static void Set(Text t, string s)
        {
            if (t != null && t.text != s) t.text = s ?? "";
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            if (!Application.isPlaying) SihReplay.RequestEditorRefresh();
        }
#endif

        public static string Pretty(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            var p = s.Replace('_', ' ');
            return char.ToUpper(p[0]) + p.Substring(1);
        }
    }
}
