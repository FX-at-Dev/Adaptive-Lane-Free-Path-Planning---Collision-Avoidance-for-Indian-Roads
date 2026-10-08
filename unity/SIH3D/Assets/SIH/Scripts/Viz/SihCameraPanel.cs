// The HUD's camera panel (V3): what the ego's camera saw at the last step,
// with the boxes its detector reported to the stack. Shown while the camera
// feeding the stack is Unity's. The boxes are a fixed set of frames in the
// scene; boxes beyond them are not drawn.

using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace Sih
{
    public class SihCameraPanel : MonoBehaviour
    {
        public SihSensorRig sensors;
        public SihLiveLink liveLink;
        [Tooltip("Shown and hidden as a whole.")]
        public GameObject content;
        public RawImage view;
        [Tooltip("The image area the boxes are placed in (pixels map onto it).")]
        public RectTransform boxArea;
        public RectTransform[] frames = new RectTransform[0];
        public Text[] labels = new Text[0];
        public Text caption;
        [Tooltip("Shows or hides the panel.")]
        public Key toggle = Key.N;
        public bool show = true;
        [Tooltip("Label text: {0} class, {1} visible fraction, {2} agent id.")]
        public string labelFormat = "{0} {1:0%}";
        public Color[] classColors = new Color[0];
        public Color defaultColor = Palette.Hex("#facc15");
        [Tooltip("Boxes from a capture older than this, real seconds, are hidden (paused, offline, restarted).")]
        public float staleAfter = 0.5f;

        void Update()
        {
            var kb = Keyboard.current;
            if (kb != null && toggle != Key.None && kb[toggle].wasPressedThisFrame) show = !show;
            bool on = show && Application.isPlaying && sensors != null && liveLink != null &&
                      liveLink.isActiveAndEnabled && liveLink.useLive && sensors.cameraSource == SihSensorRig.Source.Unity;
            if (content != null && content.activeSelf != on) content.SetActive(on);
            if (!on) return;
            if (view != null && view.texture != sensors.view) view.texture = sensors.view;

            var area = boxArea != null ? boxArea : (RectTransform)transform;
            var size = area.rect.size;
            float sx = size.x / Mathf.Max(1, sensors.imageWidth), sy = size.y / Mathf.Max(1, sensors.imageHeight);
            // The image and its boxes are one capture, so they always agree;
            // they are hidden only when captures have stopped coming (offline,
            // restarted, finished), not while paused on the last one.
            bool fresh = liveLink.running && !liveLink.done && sensors.captureWallTime >= 0f &&
                         (liveLink.paused || Time.unscaledTime - sensors.captureWallTime <= staleAfter);
            int n = fresh ? Mathf.Min(frames.Length, sensors.boxes.Count) : 0;
            for (int i = 0; i < frames.Length; i++)
            {
                var f = frames[i];
                if (f == null) continue;
                bool used = i < n;
                if (f.gameObject.activeSelf != used) f.gameObject.SetActive(used);
                if (!used) continue;
                var b = sensors.boxes[i];
                f.anchorMin = f.anchorMax = new Vector2(0, 1);
                f.pivot = new Vector2(0, 1);
                f.anchoredPosition = new Vector2(b.px.xMin * sx, -b.px.yMin * sy);
                f.sizeDelta = new Vector2(b.px.width * sx, b.px.height * sy);
                var c = b.cls >= 0 && b.cls < classColors.Length ? classColors[b.cls] : defaultColor;
                foreach (var img in f.GetComponentsInChildren<Image>(true)) if (img.color != c) img.color = c;
                if (i < labels.Length && labels[i] != null)
                {
                    string cls = b.cls >= 0 && b.cls < sensors.classes.Length ? sensors.classes[b.cls] : "?";
                    try { labels[i].text = string.Format(labelFormat, cls, b.score, b.agentId); }
                    catch (System.FormatException) { labels[i].text = cls; }
                    labels[i].color = c;
                }
            }
            if (caption != null)
                caption.text = $"CAMERA  {sensors.cameraBoxes} boxes · LiDAR {sensors.lidarPoints} returns · radar {sensors.radarReturns} · capture {sensors.captureMs:0.0} ms";
        }
    }
}
