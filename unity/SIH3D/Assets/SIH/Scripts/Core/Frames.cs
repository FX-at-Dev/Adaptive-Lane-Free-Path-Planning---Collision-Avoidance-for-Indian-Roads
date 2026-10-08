// The one place where the stack's frame becomes Unity's.
//
// On the wire everything is in the Octave world frame: x east, y north,
// heading psi counter-clockwise from +x, metres and radians. Unity is left
// handed with y up, so a world point (x, y) sits at (x, h, y) and a heading
// psi becomes a yaw of 90 - psi degrees about Unity's up axis. Nothing else
// in the project converts frames; everything goes through here.

using UnityEngine;

namespace Sih
{
    public static class Frames
    {
        public static Vector3 Pos(float x, float y, float h = 0f) => new Vector3(x, h, y);

        public static float YawDeg(float psi) => 90f - psi * Mathf.Rad2Deg;

        public static Quaternion Rot(float psi) => Quaternion.Euler(0f, YawDeg(psi), 0f);

        /// The ego pose is its rear axle; its body is centred this far ahead.
        public static Vector3 EgoCentre(float x, float y, float psi, float rearToCentre, float h = 0f)
        {
            return Pos(x + Mathf.Cos(psi) * rearToCentre, y + Mathf.Sin(psi) * rearToCentre, h);
        }

        /// Shortest-way interpolation of a heading in radians.
        public static float LerpAngle(float a, float b, float u)
        {
            float d = Mathf.Repeat(b - a + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
            return a + d * u;
        }
    }
}
