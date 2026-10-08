// Point at an object and press I: what the stack believes about it, live.
//
// Picks the tracked object whose footprint centre is nearest the mouse on
// screen (in the main view, or in Car View when that is full screen) and
// shows its id, class, speed, whether it is still and in the path, its
// footprint (size and orientation) and its track status, updated as it
// changes. Press I away from any object, or Escape, to close it.

using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace Sih
{
    public class SihInspector : MonoBehaviour
    {
        public SihReplay replay;
        [Tooltip("The view to pick in when Car View is not full screen.")]
        public Camera mainCamera;
        public SihCarView carView;
        public GameObject panel;
        public Text text;
        public Key inspectKey = Key.I;
        [Tooltip("How near the mouse an object's centre must be on screen, pixels.")]
        public float pickRadius = 70f;

        [Header("Live (read only)")]
        public int selectedId = -1;

        void Update()
        {
            var kb = Keyboard.current;
            if (kb != null && kb[inspectKey].wasPressedThisFrame) Pick();
            if (kb != null && kb.escapeKey.wasPressedThisFrame) selectedId = -1;
            Show();
        }

        Think Current(out float t)
        {
            t = replay != null ? replay.t : 0f;
            var run = replay != null ? replay.Run : null;
            if (run == null || run.frames.Count == 0) return null;
            int fi = run.FrameAt(t, out _);
            return fi >= 0 ? run.frames[fi].think : null;
        }

        void Pick()
        {
            var k = Current(out _);
            var cam = carView != null && carView.fullScreen ? carView.GetComponent<Camera>() : mainCamera;
            var mouse = Mouse.current;
            if (k == null || cam == null || mouse == null) { selectedId = -1; return; }
            var m = mouse.position.ReadValue();
            float best = pickRadius;
            int id = -1;
            foreach (var tr in k.tracks)
            {
                var sp = cam.WorldToScreenPoint(Frames.Pos(tr.x, tr.y, 0.8f));
                if (sp.z <= 0f) continue;
                float d = Vector2.Distance(m, new Vector2(sp.x, sp.y));
                if (d < best) { best = d; id = tr.id; }
            }
            selectedId = id;
        }

        void Show()
        {
            bool on = selectedId >= 0;
            if (panel != null && panel.activeSelf != on) panel.SetActive(on);
            if (!on || text == null) return;
            var k = Current(out float t);
            if (k == null) return;
            foreach (var tr in k.tracks)
            {
                if (tr.id != selectedId) continue;
                string status = tr.status == 0 ? "tentative" : tr.status == 2 ? "coasting (not seen this scan)"
                              : tr.status == 3 ? "being checked: the car slows for it, stops only once it is confirmed" : "confirmed";
                var sb = new StringBuilder();
                sb.Append($"<b>#{tr.id} {tr.cls}</b>   t {t:0.0} s\n");
                sb.Append($"{status}\n");
                sb.Append(tr.still ? "still\n" : $"moving {tr.v:0.0} m/s, heading {tr.psi * Mathf.Rad2Deg:0}°\n");
                sb.Append(tr.inPath ? "<color=#f87171><b>IN THE PATH</b></color>\n" : "not in the path\n");
                if (tr.HasFootprint)
                    sb.Append($"footprint {tr.L:0.0} × {tr.W:0.0} m, axis {tr.theta * Mathf.Rad2Deg:0}°\n");
                sb.Append($"at ({tr.x:0.0}, {tr.y:0.0}), σ {Mathf.Sqrt(Mathf.Max(0f, tr.pxx + tr.pyy)):0.00} m");
                text.text = sb.ToString();
                return;
            }
            text.text = $"#{selectedId}: no longer tracked";
        }
    }
}
