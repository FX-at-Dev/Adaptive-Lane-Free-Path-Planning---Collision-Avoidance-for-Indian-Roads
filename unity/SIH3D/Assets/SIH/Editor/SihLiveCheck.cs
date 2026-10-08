// Checks the live link end to end, without entering Play: starts the stack
// in Octave, drives a whole scenario through the real protocol with this
// project's plant (SihEgoPlant) as fast as Octave can go, then has Octave run
// the same scenario offline and compare the two trajectories.
//
// With the plant on the stack's parameters the two runs must match to the
// last few bits: that shows the bridge, the C# plant and the frame
// conversion add nothing to what the stack does. It also reports how fast
// the stack steps, which is what limits the live real-time factor.
//
// SIH > Check Live Link runs it on the open scene's scenario; batch mode:
//   Unity.exe -batchmode -projectPath <p> -executeMethod SihEditor.SihLiveCheck.Batch
//             -sihScenario village_road -sihRepo <repo> [-sihOctave <octave-cli.exe>]

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Sih;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace SihEditor
{
    public static class SihLiveCheck
    {
        [MenuItem("SIH/Check Live Link", false, 40)]
        static void Menu()
        {
            var link = UnityEngine.Object.FindFirstObjectByType<SihLiveLink>();
            if (link == null)
            {
                EditorUtility.DisplayDialog("Check Live Link", "Open a scenario scene first (it has a Live Link).", "OK");
                return;
            }
            if (!EditorUtility.DisplayDialog("Check Live Link",
                    $"Runs {link.scenario} through the live link as fast as Octave goes, then offline, and compares them. " +
                    "This takes a few minutes and Unity is busy meanwhile.", "Run", "Cancel"))
                return;
            string report = Run(link.scenario, link.repoRoot, link.octaveExe, link.port + 11, true);
            EditorUtility.ClearProgressBar();
            EditorUtility.DisplayDialog("Check Live Link", report, "OK");
        }

        public static void Batch()
        {
            string scn = Arg("-sihScenario", "village_road");
            string repo = Arg("-sihRepo", "");
            string oct = Arg("-sihOctave", "");
            if (string.IsNullOrEmpty(oct))
            {
                string local = Environment.GetEnvironmentVariable("LOCALAPPDATA") ?? "";
                foreach (var d in Directory.GetDirectories(Path.Combine(local, "Programs", "GNU Octave")))
                {
                    var exe = Path.Combine(d, "mingw64", "bin", "octave-cli.exe");
                    if (File.Exists(exe)) oct = exe;
                }
            }
            int code = 0;
            try { Debug.Log(Run(scn, repo, oct, 47611, false)); }
            catch (Exception e) { Debug.LogException(e); code = 1; }
            if (Application.isBatchMode) EditorApplication.Exit(code);
        }

        static string Arg(string name, string dflt)
        {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++) if (a[i] == name) return a[i + 1];
            return dflt;
        }

        /// Drive the scenario live, unpaced; then compare with the offline run.
        public static string Run(string scenario, string repo, string octaveExe, int port, bool progress)
        {
            if (!File.Exists(octaveExe)) throw new Exception("octave-cli not found: " + octaveExe);
            if (!File.Exists(Path.Combine(repo, "startup.m"))) throw new Exception("repository not found: " + repo);
            string root = repo.Replace('\\', '/');
            string csv = Path.Combine(Path.GetTempPath(), $"sih_live_{scenario}.csv").Replace('\\', '/');

            var listener = new TcpListener(IPAddress.Loopback, port);
            listener.Start();
            var octave = Start(octaveExe, repo,
                $"cd('{root}'); startup; sih_cosim_serve(struct('port', {port}, 'once', true, 'idle_exit_s', 120))");
            var go = new GameObject("SIH live check plant") { hideFlags = HideFlags.HideAndDontSave };
            var plant = go.AddComponent<SihEgoPlant>();
            var sb = new StringBuilder("k,t,x,y,psi,v,delta,a,state,reached,collided\n");
            var tickMs = new List<double>();
            double maxErr = 0;
            int steps = 0;
            bool reached = false, collided = false;
            try
            {
                var accept = listener.BeginAcceptTcpClient(null, null);
                if (!accept.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(90)))
                    throw new Exception("Octave did not connect within 90 s");
                using var client = listener.EndAcceptTcpClient(accept);
                client.NoDelay = true;
                var s = client.GetStream();

                var m = SihLiveLink.Receive(s, out _);
                if (Json.S(m, "type") != "ready") throw new Exception("expected ready, got " + Json.S(m, "type"));
                Write(s, new Dictionary<string, object> { ["type"] = "start", ["scenario"] = scenario, ["think"] = true }, null);
                m = SihLiveLink.Receive(s, out var hb);
                if (Json.S(m, "type") != "hello") throw new Exception("start failed: " + Json.S(m, "message"));
                var p = Json.Get(m, "plant");
                plant.wheelbase = Json.D(p, "wheelbase"); plant.aMax = Json.D(p, "a_max");
                plant.aEmergency = Json.D(p, "a_emergency"); plant.deltaMax = Json.D(p, "delta_max");
                plant.deltaRate = Json.D(p, "delta_rate");
                plant.SetState(SihLiveLink.Doubles(hb, 0, 6));
                double dt = Json.D(Json.Get(Json.Get(m, "run"), "meta"), "dt", 0.05);
                int n = (int)Json.D(m, "n_steps");

                var sw = new Stopwatch();
                for (int k = 1; k <= n; k++)
                {
                    var st = plant.State;
                    var blob = new byte[48];
                    for (int i = 0; i < 6; i++) BitConverter.GetBytes(st[i]).CopyTo(blob, 8 * i);
                    sw.Restart();
                    Write(s, new Dictionary<string, object> { ["type"] = "tick", ["k"] = (double)k }, blob);
                    var act = SihLiveLink.Receive(s, out var ab);
                    tickMs.Add(sw.Elapsed.TotalMilliseconds);
                    if (Json.S(act, "type") != "act")
                        throw new Exception($"step {k}: {Json.S(act, "message")} ({Json.S(act, "at")})");
                    var b = SihLiveLink.Doubles(ab, 0, 8);
                    plant.Step(b[0], b[1], dt);
                    var ps = plant.State;
                    for (int i = 0; i < 6; i++)
                    {
                        double d = i == 2 ? SihEgoPlant.WrapPi(ps[i] - b[2 + i]) : ps[i] - b[2 + i];
                        maxErr = Math.Max(maxErr, Math.Abs(d));
                    }
                    reached = Json.D(act, "reached") > 0.5;
                    collided = Json.D(act, "collided") > 0.5;
                    var ser = Json.Get(act, "series");
                    sb.Append(string.Join(",", k.ToString(CultureInfo.InvariantCulture),
                        R(Json.D(act, "t")), R(ps[0]), R(ps[1]), R(ps[2]), R(ps[3]), R(ps[4]), R(ps[5]),
                        Json.S(ser, "state"), reached ? "1" : "0", collided ? "1" : "0")).Append('\n');
                    steps = k;
                    if (progress && k % 20 == 0)
                        EditorUtility.DisplayProgressBar("Check Live Link", $"live: step {k} of up to {n}", (float)k / n);
                    if (Json.D(act, "done") > 0.5) break;
                }
                Write(s, new Dictionary<string, object> { ["type"] = "bye" }, null);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(go);
                listener.Stop();
                try { if (!octave.WaitForExit(5000)) octave.Kill(); } catch { }
            }
            File.WriteAllText(csv, sb.ToString());

            tickMs.Sort();
            double med = tickMs[tickMs.Count / 2], p95 = tickMs[(int)(tickMs.Count * 0.95)], mean = 0;
            foreach (var x in tickMs) mean += x;
            mean /= tickMs.Count;
            string live = $"live: {steps} steps, reached {reached}, collided {collided}\n" +
                          $"step time (Unity round trip) median {med:0.0} ms, p95 {p95:0.0} ms, mean {mean:0.0} ms " +
                          $"-> unpaced real-time factor {50.0 / mean:0.00}\n" +
                          $"plant vs Octave's own bicycle step: max difference {maxErr:0.###e+0}\n";
            Debug.Log("SIHCHECK " + live.Replace('\n', ' '));

            // The same scenario offline, compared.
            if (progress) EditorUtility.DisplayProgressBar("Check Live Link", "offline run and comparison", 1f);
            var cmp = Start(octaveExe, repo, $"cd('{root}'); startup; sih_cosim_compare('{scenario}', '{csv}')");
            string outp = cmp.StandardOutput.ReadToEnd();
            cmp.WaitForExit();
            var lines = new StringBuilder();
            foreach (var l in outp.Split('\n'))
                if (l.StartsWith("compare")) lines.Append(l.Trim()).Append('\n');
            Debug.Log("SIHCHECK " + lines.ToString().Replace('\n', ' '));
            return live + lines;
        }

        static string R(double v) => v.ToString("R", CultureInfo.InvariantCulture);

        static void Write(NetworkStream s, Dictionary<string, object> msg, byte[] blob)
        {
            var b = SihLiveLink.Pack(msg, blob);
            s.Write(b, 0, b.Length);
        }

        static Process Start(string exe, string dir, string eval)
        {
            var psi = new ProcessStartInfo(exe, $"--no-gui --eval \"{eval}\"")
            {
                WorkingDirectory = dir,
                UseShellExecute = false,
                CreateNoWindow = true,
                RedirectStandardOutput = true,
                RedirectStandardError = false,
            };
            var p = Process.Start(psi);
            SihProcess.RunAtFullSpeed(p);
            return p;
        }
    }
}
