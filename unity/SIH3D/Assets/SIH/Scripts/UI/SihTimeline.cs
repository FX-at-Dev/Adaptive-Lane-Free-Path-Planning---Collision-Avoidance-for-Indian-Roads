// The timeline strip in the footer: click or drag on it to jump. The band
// image (state, speed and clearance over the run) is a texture asset baked
// from the settings below: change them, then press "Bake band picture" on
// this component (or SIH > Rebake Timelines for every open scene).

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Sih
{
    public class SihTimeline : MonoBehaviour, IPointerDownHandler, IDragHandler, IPointerUpHandler
    {
        public SihReplay replay;
        [Tooltip("The clickable strip; the playhead spans its width.")]
        public RectTransform track;
        public RectTransform playhead;
        public RawImage band;

        [Header("Band picture (press Bake band picture after changing)")]
        public int bandWidth = 1024;
        public int bandHeight = 48;
        public Color bandBackground = Palette.Bg;
        public Color speedLineColor = Palette.Ink;
        public Color clearanceLineColor = Palette.Look;
        [Tooltip("Clearance at the top of the band, metres.")]
        public float clearanceScale = 5f;
        [Tooltip("Height of the state strip along the top, as a fraction of the band.")]
        [Range(0.05f, 1f)] public float stateStripFraction = 0.2f;
        [Range(0, 1)] public float stateStripAlpha = 0.85f;
        [Tooltip("The state colour behind the lines.")]
        [Range(0, 1)] public float stateBackgroundAlpha = 0.16f;
        public SihHud.StateColor[] stateColors =
        {
            new SihHud.StateColor("CRUISE", Palette.State["CRUISE"]),
            new SihHud.StateColor("FOLLOW", Palette.State["FOLLOW"]),
            new SihHud.StateColor("NUDGE", Palette.State["NUDGE"]),
            new SihHud.StateColor("YIELD", Palette.State["YIELD"]),
            new SihHud.StateColor("CREEP", Palette.State["CREEP"]),
            new SihHud.StateColor("STOP", Palette.State["STOP"]),
        };
        public Color unknownStateColor = Palette.Dim;

        public void Show(float u)
        {
            if (playhead == null) return;
            var a = new Vector2(Mathf.Clamp01(u), playhead.anchorMin.y);
            var b = new Vector2(Mathf.Clamp01(u), playhead.anchorMax.y);
            if (playhead.anchorMin != a) playhead.anchorMin = a;
            if (playhead.anchorMax != b) playhead.anchorMax = b;
        }

        public void OnPointerDown(PointerEventData e)
        {
            if (replay == null) return;
            replay.BeginScrub();
            Jump(e);
        }

        public void OnDrag(PointerEventData e) => Jump(e);

        public void OnPointerUp(PointerEventData e)
        {
            if (replay != null) replay.EndScrub();
        }

        void Jump(PointerEventData e)
        {
            var r = track != null ? track : (RectTransform)transform;
            if (replay == null) return;
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(r, e.position, e.pressEventCamera, out var p)) return;
            replay.SeekNormalized(Mathf.InverseLerp(r.rect.xMin, r.rect.xMax, p.x));
        }

        Color OfState(string s)
        {
            foreach (var sc in stateColors)
                if (sc != null && sc.state == s) return sc.color;
            // A state newer than the list saved in the scene (REVERSE).
            if (s != null && Palette.State.TryGetValue(s, out var pc)) return pc;
            return unknownStateColor;
        }

        /// The band picture for a run: state along the top, dimmed state
        /// behind, the speed line and the clearance line. Size is
        /// bandWidth x bandHeight.
        public Color32[] BakeBand(RunData run)
        {
            int w = Mathf.Max(2, bandWidth), h = Mathf.Max(8, bandHeight);
            var speedColor = (Color32)speedLineColor;
            var clearanceColor = (Color32)clearanceLineColor;
            var px = new Color32[w * h];
            var bg = (Color32)bandBackground;
            for (int i = 0; i < px.Length; i++) px[i] = bg;
            int n = run.sT.Length;
            if (n == 0) return px;
            int top = Mathf.Clamp(Mathf.RoundToInt(h * stateStripFraction), 2, h - 4);
            for (int x = 0; x < w; x++)
            {
                int i = Mathf.Clamp(Mathf.RoundToInt(x / (w - 1f) * (n - 1)), 0, n - 1);
                string st = i < run.sState.Length ? run.sState[i] : "";
                var c = (Color32)Palette.A(OfState(st), stateStripAlpha);
                for (int y = h - top; y < h; y++) px[y * w + x] = c;   // texture y is up
                // The state colour mixed into the background, so the background colour shows.
                var dim = (Color32)Color.Lerp(bandBackground, Palette.A(OfState(st), bandBackground.a), stateBackgroundAlpha);
                for (int y = 0; y < h - top; y++) px[y * w + x] = dim;
                int span = h - top - 3;
                int yv = Mathf.Clamp(Mathf.RoundToInt(run.sV[i] / Mathf.Max(run.vMax, 1f) * span), 0, span);
                px[yv * w + x] = speedColor;
                if (yv + 1 <= span) px[(yv + 1) * w + x] = speedColor;
                float mc = i < run.sMinClear.Length ? run.sMinClear[i] : 999f;
                if (mc < 998f)
                {
                    int yc = Mathf.Clamp(Mathf.RoundToInt(Mathf.Clamp01(mc / Mathf.Max(clearanceScale, 0.1f)) * span), 0, span);
                    px[yc * w + x] = clearanceColor;
                }
            }
            return px;
        }
    }
}
