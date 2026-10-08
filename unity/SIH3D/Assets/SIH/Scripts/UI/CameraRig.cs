// Three views in one camera: chase (behind the ego, turning with it), top
// (high and looking down along the ego's heading) and orbit (free, around the
// ego). Left or right drag orbits, middle drag pans the orbit, the wheel
// zooms. Each view's distance, pitch, field of view and look point are fields
// here, so the framing is tuned in the inspector.

using UnityEngine;
using UnityEngine.InputSystem;

namespace Sih
{
    [RequireComponent(typeof(Camera))]
    public class CameraRig : MonoBehaviour
    {
        public enum Mode { Chase, Top, Orbit }
        public static readonly string[] ModeName = { "chase", "top", "orbit" };

        [System.Serializable]
        public class View
        {
            [Tooltip("Metres from the look point.")] public float distance = 15f;
            [Tooltip("Degrees down from the horizon.")] public float pitch = 15f;
            [Tooltip("Chase/top: degrees added to the ego's heading. Orbit: absolute yaw.")] public float yaw;
            [Tooltip("Metres ahead of the ego the camera looks at.")] public float lookAhead;
            [Tooltip("Metres above the ground the camera looks at.")] public float lookHeight = 1f;
            public float fieldOfView = 55f;
        }

        [Header("View")]
        public Mode mode = Mode.Chase;
        public View chase = new View { distance = 15f, pitch = 15f, yaw = 0f, lookAhead = 7f, lookHeight = 1.4f, fieldOfView = 55f };
        public View top = new View { distance = 55f, pitch = 82f, yaw = 0f, lookAhead = 12f, lookHeight = 0f, fieldOfView = 45f };
        public View orbit = new View { distance = 30f, pitch = 38f, yaw = 30f, lookAhead = 0f, lookHeight = 1f, fieldOfView = 55f };

        [Header("Smoothing (per second)")]
        public float followSharpness = 8f;
        public float headingSharpness = 2.5f;

        [Header("Mouse")]
        public float orbitDegreesPerPixelX = 0.18f;
        public float orbitDegreesPerPixelY = 0.12f;
        public float maxPitchOffset = 60f;
        public float panPerPixel = 0.0025f;
        [Tooltip("Panning speed never drops below what it is at this distance.")] public float minPanDistance = 5f;
        [Tooltip("Fraction of the distance per wheel notch.")] public float zoomStep = 0.12f;
        [Tooltip("Smallest zoom per wheel notch, metres.")] public float minZoomStep = 1f;
        public float minDistance = 4f, maxDistance = 250f;
        public float minPitch = 3f, maxPitch = 89.5f;

        [Header("Editor")]
        [Tooltip("Move this camera when scrubbing the replay's preview slider in edit mode.")]
        public bool followInEditorPreview = true;

        [Header("Live (read only): your mouse changes, per view (chase, top, orbit)")]
        public float[] dYaw = new float[3];
        public float[] dPitch = new float[3];
        public float[] dDist = new float[3];
        public Vector3 pan;
        public Vector3 smoothTarget;
        public float smoothHeading;

        Camera cam;
        bool primed;

        public Camera Cam => cam != null ? cam : (cam = GetComponent<Camera>());

        View Current => mode == Mode.Chase ? chase : mode == Mode.Top ? top : orbit;

        void Fit()
        {
            if (dYaw == null || dYaw.Length != 3) dYaw = new float[3];
            if (dPitch == null || dPitch.Length != 3) dPitch = new float[3];
            if (dDist == null || dDist.Length != 3) dDist = new float[3];
        }

        public void ResetView()
        {
            Fit();
            for (int i = 0; i < 3; i++) { dYaw[i] = 0; dPitch[i] = 0; dDist[i] = 0; }
            pan = Vector3.zero;
            primed = false;
        }

        public void Cycle() { mode = (Mode)(((int)mode + 1) % 3); primed = false; }

        public void HandleInput(bool overUi)
        {
            var m = Mouse.current;
            if (m == null || overUi) return;
            Fit();
            int i = (int)mode;
            var v = Current;
            Vector2 d = m.delta.ReadValue();
            if (m.leftButton.isPressed || m.rightButton.isPressed)
            {
                dYaw[i] += d.x * orbitDegreesPerPixelX;
                dPitch[i] = Mathf.Clamp(dPitch[i] - d.y * orbitDegreesPerPixelY, -maxPitchOffset, maxPitchOffset);
            }
            if (m.middleButton.isPressed && mode == Mode.Orbit)
            {
                var r = transform.right; r.y = 0;
                var f = transform.forward; f.y = 0;
                float k = panPerPixel * Mathf.Max(minPanDistance, v.distance + dDist[i]);
                pan -= (r.normalized * d.x + f.normalized * d.y) * k;
            }
            float s = m.scroll.ReadValue().y;
            if (Mathf.Abs(s) > 0.01f)
            {
                float step = Mathf.Sign(s) * Mathf.Max(minZoomStep, (v.distance + dDist[i]) * zoomStep);
                dDist[i] = Mathf.Clamp(dDist[i] - step, minDistance - v.distance, maxDistance - v.distance);
            }
        }

        /// target: the ego's centre; psi: its heading. snap skips smoothing.
        public void Follow(Vector3 target, float psi, float dt, bool snap = false)
        {
            Fit();
            int i = (int)mode;
            var v = Current;
            float heading = Frames.YawDeg(psi);
            if (!primed || snap)
            {
                smoothTarget = target;
                smoothHeading = heading;
                primed = true;
            }
            smoothTarget = Vector3.Lerp(smoothTarget, target, 1f - Mathf.Exp(-dt * followSharpness));
            smoothHeading = Mathf.LerpAngle(smoothHeading, heading, 1f - Mathf.Exp(-dt * headingSharpness));

            float p = Mathf.Clamp(v.pitch + dPitch[i], minPitch, maxPitch);
            float d = v.distance + dDist[i];
            float y;
            Vector3 look = smoothTarget + Vector3.up * v.lookHeight;
            if (mode == Mode.Orbit)
            {
                y = v.yaw + dYaw[i];
                look += pan;
            }
            else
            {
                y = smoothHeading + v.yaw + dYaw[i];
                look += Quaternion.Euler(0, smoothHeading, 0) * Vector3.forward * v.lookAhead;
            }
            var rot = Quaternion.Euler(p, y, 0);
            transform.SetPositionAndRotation(look - rot * Vector3.forward * d, rot);
            Cam.fieldOfView = v.fieldOfView;
        }
    }
}
