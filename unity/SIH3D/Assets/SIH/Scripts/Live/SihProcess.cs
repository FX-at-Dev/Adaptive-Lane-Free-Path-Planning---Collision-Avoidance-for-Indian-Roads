// Windows runs a windowless background process -- Octave serving the stack --
// with "EcoQoS" power throttling, which on hybrid CPUs (performance plus
// efficiency cores) can put it on the slow cores. Measured on the demo laptop
// (i7-13650HX) that made each stack step about 3x slower: 77-98 ms against
// 28 ms. This opts the process out, documented Windows API, and raises its
// priority a step, so the live loop keeps up with real time.

using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Sih
{
    public static class SihProcess
    {
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        [StructLayout(LayoutKind.Sequential)]
        struct PowerThrottling { public uint Version, ControlMask, StateMask; }

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool SetProcessInformation(System.IntPtr process, int infoClass, ref PowerThrottling info, int size);

        const int ProcessPowerThrottling = 4;
        const uint ExecutionSpeed = 1;
#endif

        /// Full speed for p: no power throttling, above-normal priority.
        /// Returns false where that is not possible (and changes nothing).
        public static bool RunAtFullSpeed(Process p)
        {
            if (p == null) return false;
            bool ok = false;
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            try
            {
                var s = new PowerThrottling { Version = 1, ControlMask = ExecutionSpeed, StateMask = 0 };
                ok = SetProcessInformation(p.Handle, ProcessPowerThrottling, ref s, Marshal.SizeOf(typeof(PowerThrottling)));
            }
            catch { ok = false; }
#endif
            try { p.PriorityClass = ProcessPriorityClass.AboveNormal; } catch { }
            return ok;
        }
    }
}
