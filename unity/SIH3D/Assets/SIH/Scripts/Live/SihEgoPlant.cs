// The ego vehicle's plant in the live closed loop (V2): the stack in Octave
// decides an acceleration and a steering angle every 50 ms, and this moves
// the car.
//
// It is a line-for-line port of core/vehicle/sih_bicycle_step.m -- actuator
// limits, then RK4 on the rear-axle kinematic bicycle with the speed ramping
// across the step -- in double precision, so with the stack's parameters it
// lands where Octave's own step does to the last few bits. Octave reports the
// difference back every step (Plant error below). With "Use Stack Parameters"
// off, the values here are used instead: a deliberately different plant, to
// see how the controller copes with a vehicle it was not tuned for.

using UnityEngine;

namespace Sih
{
    public class SihEgoPlant : MonoBehaviour
    {
        [Header("Parameters")]
        [Tooltip("Take the parameters below from the stack's config when the run starts (exact plant). Off: use the values as set here.")]
        public bool useStackParameters = true;
        [Tooltip("Metres between the axles.")]
        public double wheelbase = 2.70;
        [Tooltip("Strongest acceleration, m/s².")]
        public double aMax = 2.50;
        [Tooltip("Hardest braking, m/s² (negative).")]
        public double aEmergency = -7.00;
        [Tooltip("Steering angle limit, radians.")]
        public double deltaMax = 0.55;
        [Tooltip("Steering rate limit, radians per second.")]
        public double deltaRate = 0.60;
        [Tooltip("Fastest it backs up, m/s.")]
        public double vReverseMax = 1.5;

        [Header("Live state (read only)")]
        [Tooltip("Rear axle, world frame: x east, y north, metres.")]
        public double x;
        public double y;
        [Tooltip("Heading, radians counter-clockwise from east.")]
        public double psi;
        [Tooltip("Speed, m/s.")]
        public double v;
        [Tooltip("Steering angle, radians.")]
        public double delta;
        [Tooltip("Acceleration actually applied, m/s².")]
        public double a;
        [Tooltip("Last command from the stack: acceleration, steering.")]
        public double aCmd, deltaCmd;

        public void SetState(double[] s)
        {
            x = s[0]; y = s[1]; psi = s[2]; v = s[3]; delta = s[4]; a = s[5];
        }

        public double[] State => new[] { x, y, psi, v, delta, a };

        [Tooltip("Gear of the last step: +1 forward, -1 reverse.")]
        public int gear = 1;

        /// Advance one step of dt seconds under the command (a, delta) in
        /// gear +1 (forward) or -1 (reverse): sih_bicycle_step, exactly.
        public void Step(double aCommand, double deltaCommand, double dt, int gearCommand = 1)
        {
            gear = gearCommand >= 0 ? 1 : -1;
            aCmd = aCommand; deltaCmd = deltaCommand;
            double L = wheelbase;

            // ---- actuator limits ----
            double acc = System.Math.Min(System.Math.Max(aCommand, aEmergency), aMax);
            double dTarget = System.Math.Min(System.Math.Max(deltaCommand, -deltaMax), deltaMax);
            double dStep = deltaRate * dt;
            double d = delta + System.Math.Min(System.Math.Max(dTarget - delta, -dStep), dStep);
            d = System.Math.Min(System.Math.Max(d, -deltaMax), deltaMax);

            // The speed may not cross zero within a gear, nor back up faster
            // than vReverseMax.
            double v0 = v;
            if (gear >= 0)
            {
                if (v0 + acc * dt < 0) acc = -v0 / dt;
            }
            else
            {
                if (v0 + acc * dt > 0) acc = -v0 / dt;
                else if (v0 + acc * dt < -vReverseMax) acc = (-vReverseMax - v0) / dt;
            }

            // ---- RK4 on (x, y, psi), v(t) = v0 + a t, delta constant ----
            double tanL = System.Math.Tan(d) / L;
            double z0x = x, z0y = y, z0p = psi;
            Deriv(z0p, 0, v0, acc, tanL, out var k1x, out var k1y, out var k1p);
            Deriv(z0p + (dt / 2) * k1p, dt / 2, v0, acc, tanL, out var k2x, out var k2y, out var k2p);
            Deriv(z0p + (dt / 2) * k2p, dt / 2, v0, acc, tanL, out var k3x, out var k3y, out var k3p);
            Deriv(z0p + dt * k3p, dt, v0, acc, tanL, out var k4x, out var k4y, out var k4p);

            x = z0x + (dt / 6) * (k1x + 2 * k2x + 2 * k3x + k4x);
            y = z0y + (dt / 6) * (k1y + 2 * k2y + 2 * k3y + k4y);
            psi = WrapPi(z0p + (dt / 6) * (k1p + 2 * k2p + 2 * k3p + k4p));
            v = gear >= 0 ? System.Math.Max(0, v0 + acc * dt) : System.Math.Min(0, v0 + acc * dt);
            delta = d;
            a = acc;
        }

        static void Deriv(double p, double tau, double v0, double acc, double tanL,
                          out double dx, out double dy, out double dp)
        {
            double vv = v0 + acc * tau;
            dx = vv * System.Math.Cos(p);
            dy = vv * System.Math.Sin(p);
            dp = vv * tanL;
        }

        /// sih_wrap_pi: mod(a + pi, 2 pi) - pi, with Octave's mod (sign of
        /// the divisor), and -pi reported as pi.
        public static double WrapPi(double ang)
        {
            const double P = System.Math.PI, P2 = 2 * System.Math.PI;
            double u = ang + P;
            double q = u / P2;
            double m = System.Math.Round(q) == q ? 0 : u - System.Math.Floor(q) * P2;   // as Octave's mod
            double r = m - P;
            return r == -P ? P : r;
        }
    }
}
