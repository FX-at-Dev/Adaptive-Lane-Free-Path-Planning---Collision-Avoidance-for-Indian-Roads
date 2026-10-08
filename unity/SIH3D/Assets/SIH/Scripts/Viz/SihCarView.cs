// Car View: exactly what the car sees, from where its camera sits.
//
// Put this on a camera (the scene's "Car View"). Every frame it moves to the
// ego's camera mount -- the ego's world position and rotation at the moment
// on screen, at the sensor's mount height -- and takes the sensor's field of
// view. It renders into a picture-in-picture panel on the HUD; the key swaps
// it to full screen and back.
//
// The stack's objects are drawn in the world as their footprints (Think
// Layers), so their boxes sit on the objects in this view exactly as in the
// main one. The panel also lists the objects in the car's field of view,
// those in its path first.

using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace Sih
{
    [ExecuteAlways]
    [RequireComponent(typeof(Camera))]
    public class SihCarView : MonoBehaviour
    {
        [Header("Where it looks from")]
        [Tooltip("The ego vehicle.")]
        public Transform ego;
        [Tooltip("Take the mount height and field of view from the ego's sensor camera.")]
        public SihSensorRig sensors;
        public bool useSensorMount = true;
        [Tooltip("Used when not taking them from the sensors: metres above the ego's centre, and ahead of it.")]
        public float mountHeight = 1.4f, mountForward = 0f;
        [Tooltip("Horizontal field of view, degrees (when not taking it from the sensors).")]
        public float horizontalFov = 100f;

        [Header("Display")]
        [Tooltip("Picture-in-picture target.")]
        public RenderTexture pipTexture;
        public RawImage pipImage;
        [Tooltip("The panel holding the picture and the object list.")]
        public GameObject pipPanel;
        [Tooltip("Swaps Car View to full screen and back.")]
        public Key fullScreenKey = Key.G;
        public bool fullScreen;
        [Tooltip("Disabled while Car View is full screen.")]
        public Camera mainCamera;

        [Header("Objects in view")]
        public SihReplay replay;
        public Text objectList;
        [Tooltip("At most this many lines.")]
        public int maxListed = 8;
        public string inPathText = "IN PATH", stillText = "still";

        Camera cam;

        Camera Cam => cam != null ? cam : (cam = GetComponent<Camera>());

        void Update()
        {
            var kb = Keyboard.current;
            if (Application.isPlaying && kb != null && fullScreenKey != Key.None && kb[fullScreenKey].wasPressedThisFrame)
                fullScreen = !fullScreen;
        }

        void LateUpdate()
        {
            if (ego == null) return;
            float h = mountHeight, fwd = mountForward, hfov = horizontalFov;
            if (useSensorMount && sensors != null) { h = sensors.cameraMount; fwd = 0f; hfov = sensors.cameraFov; }

            // The ego object stands at its centre, turned to its heading.
            transform.SetPositionAndRotation(ego.position + ego.rotation * new Vector3(0f, h, fwd), ego.rotation);

            bool full = fullScreen && Application.isPlaying;
            var target = full ? null : pipTexture;
            if (Cam.targetTexture != target) Cam.targetTexture = target;
            float aspect = full ? (float)Screen.width / Mathf.Max(1, Screen.height)
                                : (pipTexture != null ? (float)pipTexture.width / pipTexture.height : Cam.aspect);
            Cam.fieldOfView = 2f * Mathf.Atan(Mathf.Tan(hfov * 0.5f * Mathf.Deg2Rad) / Mathf.Max(aspect, 0.1f)) * Mathf.Rad2Deg;
            Cam.depth = full && mainCamera != null ? mainCamera.depth + 1 : (mainCamera != null ? mainCamera.depth - 1 : -2);
            if (Application.isPlaying && mainCamera != null && mainCamera.enabled == full) mainCamera.enabled = !full;
            if (pipPanel != null && pipPanel.activeSelf == full) pipPanel.SetActive(!full);
            if (pipImage != null && pipImage.texture != pipTexture) pipImage.texture = pipTexture;

            ListObjects();
        }

        struct Seen { public TrackView t; public float dist; }
        readonly List<Seen> seen = new List<Seen>();

        /// The stack's objects in this camera's view at the moment on screen.
        void ListObjects()
        {
            if (objectList == null) return;
            var run = replay != null ? replay.Run : null;
            if (run == null || run.frames.Count == 0) { Set(""); return; }
            int fi = run.FrameAt(replay.t, out _);
            var k = fi >= 0 ? run.frames[fi].think : null;
            if (k == null) { Set(""); return; }
            seen.Clear();
            foreach (var tr in k.tracks)
            {
                if (tr.status == 0) continue;
                var p = Frames.Pos(tr.x, tr.y, 0.8f);
                var vp = Cam.WorldToViewportPoint(p);
                if (vp.z <= 0f || vp.x < 0f || vp.x > 1f || vp.y < -0.2f || vp.y > 1.2f) continue;
                seen.Add(new Seen { t = tr, dist = vp.z });
            }
            seen.Sort((a, b) => a.t.inPath != b.t.inPath ? (a.t.inPath ? -1 : 1) : a.dist.CompareTo(b.dist));
            var sb = new StringBuilder();
            sb.Append($"IN VIEW  {seen.Count}\n");
            for (int i = 0; i < seen.Count && i < maxListed; i++)
            {
                var t = seen[i].t;
                sb.Append($"#{t.id} {t.cls}  {seen[i].dist:0} m");
                if (t.still) sb.Append("  ").Append(stillText);
                else sb.Append($"  {t.v:0.0} m/s");
                if (t.inPath) sb.Append("  <b>").Append(inPathText).Append("</b>");
                sb.Append('\n');
            }
            Set(sb.ToString());
        }

        void Set(string s)
        {
            if (objectList.text != s) objectList.text = s;
        }

        void OnDisable()
        {
            if (Application.isPlaying && mainCamera != null) mainCamera.enabled = true;
        }
    }
}
