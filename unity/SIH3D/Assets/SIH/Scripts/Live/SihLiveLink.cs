// V2: the live closed loop. The autonomy stack runs in Octave
// (cosim/sih_cosim_serve.m) and drives the ego in this scene.
//
// Every 50 ms of simulated time this sends the ego's state and what the
// sensors (SihSensorRig) saw (TICK) and gets back the stack's command and what
// it believed (ACT); the ego's plant (SihEgoPlant) applies the command. The replay shows the run as it grows,
// a little behind the newest step so motion stays smooth: the simulation may
// run up to Max Lead ahead of what is on screen, which absorbs the planner's
// slow cycles. If the stack is slower than real time on average, the picture
// slows down instead of the car driving on stale commands, and the HUD shows
// the real-time factor.
//
// Judges can drop road users into the world (keys below, at the mouse
// pointer): they join the stack's ground truth like any other agent and are
// shown by the spare road-user objects in the scene.
//
// Nothing is created in the scene. Everything is set here in the inspector.

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.InputSystem;
using Debug = UnityEngine.Debug;

namespace Sih
{
    public class SihLiveLink : MonoBehaviour
    {
        [Serializable]
        public class DropPreset
        {
            public string name = "cow";
            public Key key = Key.B;
            [Tooltip("car, bus, truck, auto, two_wheeler, bicycle, pedestrian, cattle, pushcart or static.")]
            public string agentClass = "cattle";
            [Tooltip("static: stands still. cross: walks straight along its heading.")]
            public string mode = "static";
            [Tooltip("m/s")]
            public float speed;
            [Tooltip("Heading relative to the road at that point, degrees (90 = across, to the left).")]
            public float headingFromRoad;
        }

        [Header("Mode")]
        [Tooltip("On: Play runs the stack live. Off: Play replays the recorded run file.")]
        public bool useLive = true;
        [Tooltip("Scenario the stack builds (scenarios/sih_scn_<name>.m).")]
        public string scenario = "village_road";
        [Tooltip("Record what the stack thinks each cycle, for the overlays.")]
        public bool think = true;
        [Tooltip("World seed: 0 draws a new random world every Play (and every restart). Any other number gives that world again, exactly: the same traffic and the same scenery.")]
        public int seed;
        [Tooltip("On: the stack draws random traffic that keeps the scenario's story, from the world seed. Off: the scenario's scripted traffic.")]
        public bool randomTraffic = true;
        [Tooltip("Regenerated with each new world (restart), so scenery and traffic share the seed.")]
        public SihSceneryGenerator scenery;

        [Header("Target")]
        [Tooltip("Where the car stops: move this (the scene's Goal flag) along the road before Play. The car brakes smoothly and stops there, in its own lane; it is the only stop it plans. Empty: the scenario's own goal.")]
        public Transform target;

        [Header("Connection")]
        [Tooltip("Port the stack connects to. If it is taken -- most often by this editor, holding one an earlier Play did not release -- the next free one is used (see Active Port).")]
        public int port = 47600;
        [Tooltip("How many ports from Port on to try before giving up.")]
        public int portsToTry = 10;
        [Tooltip("Live (read only): the port actually in use this Play.")]
        public int activePort;
        [Tooltip("Ping the stack this often while not stepping, seconds.")]
        public float pingInterval = 1f;

        public enum Launcher { Octave, PythonScript }

        [Header("Octave")]
        [Tooltip("Start the stack on Play if it is not already serving.")]
        public bool launchOctave = true;
        [Tooltip("Octave: Unity starts Octave and talks to it directly. Python Script: Unity starts your script (main.py), which starts Octave and relays every message -- every command the car gets passes through its drive() function.")]
        public Launcher launcher = Launcher.PythonScript;
        [Tooltip("Python Script: the interpreter.")]
        public string pythonExe = "python";
        [Tooltip("Python Script: the script, relative to Repo Root (or a full path).")]
        public string pythonScript = "main.py";
        [Tooltip("octave-cli.exe")]
        public string octaveExe = "";
        [Tooltip("The SIH repository (the folder with startup.m).")]
        public string repoRoot = "";
        public bool showOctaveWindow;
        [Tooltip("Stop Octave when Play ends. Off keeps it warm for the next Play (it quits by itself after Idle Exit).")]
        public bool stopOctaveOnExit;
        [Tooltip("Octave quits after this long without Unity, seconds.")]
        public float octaveIdleExit = 900f;
        [Tooltip("Write Octave's output to the Console.")]
        public bool logOctave = true;

        [Header("Pacing")]
        [Tooltip("Simulated seconds per real second.")]
        public float speed = 1f;
        [Tooltip("How far the simulation may run ahead of the picture, seconds.")]
        public float maxLead = 0.3f;
        [Tooltip("The picture trails the newest step by at least this, seconds, so agents can be interpolated.")]
        public float minLag = 0.1f;
        [Tooltip("Smooth playback: when the stack runs slower than real time the picture slows down evenly, staying this far behind the newest step, instead of running up to it and freezing until the next (which looks like the car stuttering back and forth). 0 turns it off.")]
        public float smoothLag = 0.2f;
        public bool paused;

        [Header("After the goal")]
        public bool restartAfterGoal = true;
        public float restartDelay = 2f;
        [Tooltip("A run in which the car has not moved for this long, seconds, is given up and a new world starts, instead of waiting out the scenario's time limit. 0: never.")]
        public float restartWhenStuckFor = 20f;
        [Tooltip("Start a new world now.")]
        public Key newWorldKey = Key.R;

        [Header("Plant")]
        public SihEgoPlant plant;

        [Header("Sensors (V3)")]
        [Tooltip("The ego's sensors in this scene. Which sensor comes from Unity, from the stack's own model, or is off is set on it.")]
        public SihSensorRig sensors;

        [Header("Judge drops (at the mouse pointer)")]
        public Camera pointerCamera;
        public DropPreset[] drops =
        {
            new DropPreset { name = "cow", key = Key.B, agentClass = "cattle", mode = "static", speed = 0f, headingFromRoad = 70f },
            new DropPreset { name = "pedestrian crossing", key = Key.P, agentClass = "pedestrian", mode = "cross", speed = 1.3f, headingFromRoad = 90f },
            new DropPreset { name = "pushcart", key = Key.K, agentClass = "pushcart", mode = "static", speed = 0f, headingFromRoad = 0f },
        };
        [Tooltip("Removes the last road user dropped.")]
        public Key removeLastDrop = Key.Backspace;

        [Header("Live (read only)")]
        public string status = "not started";
        public bool connected;
        public bool running;
        public bool done;
        public int step;
        [Tooltip("Newest simulated time, seconds.")]
        public float simTime;
        [Tooltip("Time on screen, seconds.")]
        public float displayTime;
        [Tooltip("Simulated seconds per real second, smoothed.")]
        public float realTimeFactor = 1f;
        [Tooltip("Octave's time per step, ms: world, perception, stack, frame, total.")]
        public float[] stepTiming = new float[5];
        [Tooltip("What perception made of the last capture.")]
        public string perception = "";
        [Tooltip("The finished run's metrics, as sih_run_all reports them offline.")]
        public string result = "";
        [Tooltip("Largest gap between this plant and Octave's own bicycle step so far.")]
        public double plantError;
        public bool reached, collided;
        public int dropped;
        public string octaveVersion = "";
        public string lastOctaveLine = "";

        public RunData Run { get; private set; }

        int worldSeed;
        /// The seed of the world on screen: Seed if set, else a fresh random one.
        public int WorldSeed
        {
            get
            {
                if (worldSeed == 0) worldSeed = seed != 0 ? seed : 1 + (System.Environment.TickCount & 0x3fffffff) % 999983;
                return worldSeed;
            }
        }

        /// Octave's total time for the last step, ms.
        public float StepTotalMs => stepTiming != null && stepTiming.Length > 0 ? stepTiming[stepTiming.Length - 1] : 0f;

        // ---- private plumbing -------------------------------------------------

        struct Msg { public object json; public byte[] blob; public string error; }

        TcpListener listener;
        // The listener of the last Play, kept where a new Play can find it:
        // with domain reload off on entering Play, one an earlier Play failed
        // to close is still open in the editor and holds the port.
        static TcpListener lastListener;
        TcpClient client;
        NetworkStream stream;
        Thread acceptThread, readThread;
        readonly ConcurrentQueue<Msg> inbox = new ConcurrentQueue<Msg>();
        readonly ConcurrentQueue<string> octaveOut = new ConcurrentQueue<string>();
        readonly object sendLock = new object();
        volatile bool quitting;
        Process octave;
        bool awaiting, stepOnce;
        double stepDt = 0.05;     // exact, from the stack: a float 0.05 is not 0.05
        object senseAgents;       // where the stack has every road user for the next capture
        float lastSent, doneAt = -1f, rtfClock, rtfDisplay, stuckSince = -1f;
        bool restartPending;
        volatile int lastHeard;   // Environment.TickCount when the current stack last spoke
        readonly List<object> pending = new List<object>();
        readonly List<int> myDrops = new List<int>();

        // Per project: two copies of the project (same company and product
        // name) share PlayerPrefs, and one's stack must never pass for the other's.
        static readonly string KeyTag = ProjectTag();
        static readonly string PidKey = "SIH.octavePid." + KeyTag;
        static readonly string PortKey = "SIH.octavePort." + KeyTag;     // the port that Octave connects to
        static readonly string LauncherKey = "SIH.launcher." + KeyTag;   // what was started: 0 Octave, 1 the Python script

        static string ProjectTag()
        {
            uint h = 2166136261u;                     // FNV-1a of the project's path: stable across sessions
            foreach (char ch in Application.dataPath.ToLowerInvariant()) h = (h ^ ch) * 16777619u;
            return h.ToString("x8");
        }

        // ---- lifecycle -------------------------------------------------------

        void Start()
        {
            if (!useLive) { status = "replaying the run file"; enabled = false; return; }
            if (plant == null) plant = FindFirstObjectByType<SihEgoPlant>();
            if (pointerCamera == null) pointerCamera = Camera.main;
            if (lastListener != null)
            {
                try { lastListener.Stop(); } catch { }
                lastListener = null;
            }
            // The first free port from Port on: one held by something else --
            // this editor, still holding a port a Play long ago never released --
            // is stepped over rather than ending the run.
            string lastError = "";
            for (int k = 0; k < Mathf.Max(1, portsToTry) && listener == null; k++)
            {
                try
                {
                    var l = new TcpListener(IPAddress.Loopback, port + k);
                    l.Start();
                    NotInherited(l.Server);
                    listener = l;
                    activePort = port + k;
                }
                catch (Exception e) { lastError = e.Message; }
            }
            if (listener == null)
            {
                status = $"cannot listen on ports {port}-{port + Mathf.Max(1, portsToTry) - 1}: {lastError}";
                Debug.LogError($"SIH live: {status}\nRestart Unity to free them, or set Port on the Live Link to another range.", this);
                enabled = false;
                return;
            }
            lastListener = listener;
            if (activePort != port)
                Debug.LogWarning($"SIH live: port {port} is taken (most often by this editor, holding it from an earlier Play); using {activePort}. Restarting Unity frees it.", this);
            acceptThread = new Thread(AcceptLoop) { IsBackground = true, Name = "SIH live accept" };
            acceptThread.Start();
            status = "waiting for the stack";
            if (launchOctave) LaunchOctave();
        }

        // OnDisable too: recompiling scripts during Play reloads the code
        // without OnDestroy, and the listener it skipped kept the port for
        // every later Play ("cannot listen on port 47600").
        void OnDisable() => Shutdown();
        void OnDestroy() => Shutdown();
        void OnApplicationQuit() => Shutdown();

        void Shutdown()
        {
            if (quitting) return;
            quitting = true;
            try { if (stream != null) Send(new Dictionary<string, object> { ["type"] = "bye" }, null); } catch { }
            try { client?.Close(); } catch { }
            try { listener?.Stop(); } catch { }
            if (lastListener == listener) lastListener = null;
            if (stopOctaveOnExit && octave != null)
            {
                try { if (!octave.HasExited) octave.Kill(); } catch { }
                PlayerPrefs.DeleteKey(PidKey);
            }
        }

        // Windows hands a child process a copy of every inheritable handle, and
        // a socket is one. Octave, started from here, kept a copy of the
        // listening socket and held the port long after Unity closed it; the
        // next Play found it taken. Marked not inheritable, it is Unity's alone.
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool SetHandleInformation(IntPtr handle, uint mask, uint flags);
#endif
        static void NotInherited(Socket s)
        {
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            try { SetHandleInformation(s.Handle, 1u, 0u); } catch { }   // HANDLE_FLAG_INHERIT off
#endif
        }

        // ---- Octave ----------------------------------------------------------

        void LaunchOctave()
        {
            // One left running by an earlier Play is still polling for us.
            int pid = PlayerPrefs.GetInt(PidKey, 0);
            if (pid != 0)
            {
                try
                {
                    var p = Process.GetProcessById(pid);
                    // Reused only if it connects to this port; on another one it
                    // would wait for us for ever (it idles out on its own).
                    string want = launcher == Launcher.PythonScript ? "python" : "octave";
                    if (!p.HasExited && p.ProcessName.IndexOf(want, StringComparison.OrdinalIgnoreCase) >= 0 &&
                        PlayerPrefs.GetInt(PortKey, 47600) == activePort && PlayerPrefs.GetInt(LauncherKey, 0) == (int)launcher)
                    {
                        status = "waiting for the running stack (Octave " + pid + ")";
                        return;
                    }
                    // Not reusable (another launcher or port): stop it, or it
                    // keeps polling and fights the new one for the connection.
                    string n = p.ProcessName;
                    if (!p.HasExited && (n.IndexOf("python", StringComparison.OrdinalIgnoreCase) >= 0 ||
                                         n.IndexOf("octave", StringComparison.OrdinalIgnoreCase) >= 0))
                        p.Kill();
                }
                catch { }
                PlayerPrefs.DeleteKey(PidKey);
            }
            if (string.IsNullOrEmpty(octaveExe) || !File.Exists(octaveExe))
            {
                status = "Octave not found: set Octave Exe on the Live Link, or start the stack by hand";
                Debug.LogWarning("SIH live: " + status, this);
                return;
            }
            if (string.IsNullOrEmpty(repoRoot) || !File.Exists(Path.Combine(repoRoot, "startup.m")))
            {
                status = "repository not found: set Repo Root on the Live Link";
                Debug.LogWarning("SIH live: " + status, this);
                return;
            }
            string root = repoRoot.Replace('\\', '/');
            string eval = $"cd('{root}'); startup; sih_cosim_serve(struct('port', {activePort}, 'idle_exit_s', {octaveIdleExit.ToString(CultureInfo.InvariantCulture)}))";
            string exe = octaveExe, argsLine = $"--no-gui --eval \"{eval}\"";
            if (launcher == Launcher.PythonScript)
            {
                // The script starts Octave itself and sits between it and us.
                string script = Path.IsPathRooted(pythonScript) ? pythonScript : Path.Combine(repoRoot, pythonScript);
                if (!File.Exists(script))
                {
                    status = "Python script not found: " + script;
                    Debug.LogWarning("SIH live: " + status, this);
                    return;
                }
                exe = string.IsNullOrEmpty(pythonExe) ? "python" : pythonExe;
                argsLine = $"-u \"{script}\" --unity-port {activePort} --repo \"{root}\" --octave \"{octaveExe}\"";
            }
            Launcher used = launcher;
            try
            {
                try { octave = Process.Start(StartInfo(exe, argsLine)); }
                catch (System.ComponentModel.Win32Exception) when (launcher == Launcher.PythonScript)
                {
                    // No Python: drive with Octave directly rather than not at all.
                    Debug.LogWarning($"SIH live: could not start Python ('{exe}'); driving with Octave directly. Set Python Exe on the Live Link to your python.exe to drive through {pythonScript}.", this);
                    used = Launcher.Octave;
                    octave = Process.Start(StartInfo(octaveExe, $"--no-gui --eval \"{eval}\""));
                }
                SihProcess.RunAtFullSpeed(octave);     // not on the slow cores (EcoQoS)
                if (!showOctaveWindow)
                {
                    octave.OutputDataReceived += (_, e) => { if (e.Data != null) octaveOut.Enqueue(e.Data); };
                    octave.ErrorDataReceived += (_, e) => { if (e.Data != null) octaveOut.Enqueue(e.Data); };
                    octave.BeginOutputReadLine();
                    octave.BeginErrorReadLine();
                }
                PlayerPrefs.SetInt(PidKey, octave.Id);
                PlayerPrefs.SetInt(PortKey, activePort);
                PlayerPrefs.SetInt(LauncherKey, (int)launcher);
                status = used == Launcher.PythonScript
                    ? $"starting {Path.GetFileName(pythonScript)}, which starts the stack (about 10 s the first time)"
                    : "starting Octave (about 10 s the first time)";
            }
            catch (Exception e)
            {
                status = "could not start Octave: " + e.Message;
                Debug.LogError("SIH live: " + status, this);
            }
        }

        ProcessStartInfo StartInfo(string exe, string argsLine) => new ProcessStartInfo(exe, argsLine)
        {
            WorkingDirectory = repoRoot,
            UseShellExecute = false,
            CreateNoWindow = !showOctaveWindow,
            RedirectStandardOutput = !showOctaveWindow,
            RedirectStandardError = !showOctaveWindow,
        };

        // ---- network threads -------------------------------------------------

        void AcceptLoop()
        {
            while (!quitting)
            {
                TcpClient c;
                try { c = listener.AcceptTcpClient(); }
                catch { return; }
                c.NoDelay = true;
                NotInherited(c.Client);
                // One stack at a time. A second one (left running by an earlier
                // Play, or started by hand) used to replace the first; with
                // several polling, each knocked the last one off and the run
                // went STACK OFFLINE over and over. A newcomer now takes over
                // only from a connection that has gone quiet.
                bool live;
                lock (sendLock) live = client != null && client.Connected &&
                                       unchecked(Environment.TickCount - lastHeard) < 10000;
                if (live)
                {
                    try { c.Close(); } catch { }
                    continue;
                }
                lock (sendLock)
                {
                    try { client?.Close(); } catch { }
                    client = c;
                    stream = c.GetStream();
                    lastHeard = Environment.TickCount;
                }
                readThread = new Thread(() => ReadLoop(c)) { IsBackground = true, Name = "SIH live read" };
                readThread.Start();
            }
        }

        void ReadLoop(TcpClient c)
        {
            try
            {
                var s = c.GetStream();
                while (!quitting)
                {
                    // Parsed here, off the main thread: a frame is tens of kilobytes.
                    var json = Receive(s, out var blob);
                    lastHeard = Environment.TickCount;
                    inbox.Enqueue(new Msg { json = json, blob = blob });
                }
            }
            catch (Exception e)
            {
                bool current;
                lock (sendLock) current = ReferenceEquals(c, client);
                if (!quitting && current) inbox.Enqueue(new Msg { error = e.Message });
            }
            finally
            {
                try { c.Close(); } catch { }      // or it lingers half-closed
            }
        }

        static void ReadExactly(NetworkStream s, byte[] b, int n)
        {
            int got = 0;
            while (got < n)
            {
                int r = s.Read(b, got, n - got);
                if (r <= 0) throw new IOException("the stack closed the connection");
                got += r;
            }
        }

        /// The wire form of a message: uint32 JSON length, uint32 binary
        /// length (little endian), the JSON, the binary part.
        public static byte[] Pack(Dictionary<string, object> msg, byte[] blob)
        {
            var j = Encoding.UTF8.GetBytes(JsonWrite.Write(msg));
            blob ??= Array.Empty<byte>();
            var buf = new byte[8 + j.Length + blob.Length];
            BitConverter.GetBytes(j.Length).CopyTo(buf, 0);
            BitConverter.GetBytes(blob.Length).CopyTo(buf, 4);
            j.CopyTo(buf, 8);
            blob.CopyTo(buf, 8 + j.Length);
            return buf;
        }

        /// Read one whole message (blocking).
        public static object Receive(NetworkStream s, out byte[] blob)
        {
            var head = new byte[8];
            ReadExactly(s, head, 8);
            int nj = BitConverter.ToInt32(head, 0), nb = BitConverter.ToInt32(head, 4);
            var jb = new byte[nj];
            ReadExactly(s, jb, nj);
            blob = new byte[nb];
            ReadExactly(s, blob, nb);
            return Json.Parse(Encoding.UTF8.GetString(jb));
        }

        void Send(Dictionary<string, object> msg, byte[] blob)
        {
            var buf = Pack(msg, blob);
            lock (sendLock)
            {
                if (stream == null) return;
                stream.Write(buf, 0, buf.Length);
            }
            lastSent = Time.unscaledTime;
        }

        // ---- main loop -------------------------------------------------------

        void Update()
        {
            while (octaveOut.TryDequeue(out var line))
            {
                lastOctaveLine = line;
                if (logOctave) Debug.Log("[Octave] " + line, this);
            }
            while (inbox.TryDequeue(out var m)) Handle(m);

            // An Octave that stopped before connecting (a script error, a
            // missing package) would otherwise leave this waiting for ever.
            if (octave != null && octave.HasExited)
            {
                string who = launcher == Launcher.PythonScript ? Path.GetFileName(pythonScript) : "Octave";
                status = $"{who} stopped (exit code {octave.ExitCode}){(connected ? "" : " before connecting")}. Last output: {lastOctaveLine}";
                Debug.LogError("SIH live: " + status, this);
                PlayerPrefs.DeleteKey(PidKey);
                octave = null;
            }

            HandleKeys();

            if (Run == null) return;
            float dt = Time.unscaledDeltaTime;

            // The picture advances in real time (times speed), never past what
            // the stack has produced.
            float before = displayTime;
            if (!paused)
            {
                float newest = Run.sT.Length > 0 ? Run.sT[Run.sT.Length - 1] : 0f;
                float limit = done ? newest : Mathf.Max(0f, newest - minLag);
                float rate = speed;
                if (smoothLag > 0f && !done)
                {
                    // Ease off as the picture closes on the newest step, so it
                    // moves evenly at the stack's pace rather than in bursts.
                    float ahead = newest - displayTime;
                    rate *= Mathf.Clamp((ahead - minLag) / Mathf.Max(smoothLag, 1e-3f), 0.15f, 1.25f);
                }
                displayTime = Mathf.Min(displayTime + dt * rate, limit);
                if (displayTime < before) displayTime = before;
            }
            rtfClock += dt;
            rtfDisplay += displayTime - before;
            if (rtfClock >= 0.5f)
            {
                realTimeFactor = Mathf.Lerp(realTimeFactor, rtfDisplay / rtfClock, 0.5f);
                rtfClock = 0f; rtfDisplay = 0f;
            }

            if (running && connected && !awaiting && !done && !restartPending)
            {
                bool want = (!paused && simTime - displayTime < maxLead) || stepOnce;
                if (want) SendTick();
            }
            else if (connected && Time.unscaledTime - lastSent > pingInterval)
            {
                Send(new Dictionary<string, object> { ["type"] = "ping" }, null);
            }

            if (done && restartAfterGoal)
            {
                if (doneAt < 0f && displayTime >= simTime - 1e-3f) doneAt = Time.unscaledTime;
                if (doneAt >= 0f && Time.unscaledTime - doneAt > restartDelay) Restart();
            }

            // Stuck: the car has not moved for restartWhenStuckFor seconds of
            // the run. Rather than sit out the scenario's time limit, start a
            // new world.
            if (running && !done && plant != null && restartWhenStuckFor > 0f)
            {
                if (System.Math.Abs(plant.v) > 0.05 || stuckSince < 0f) stuckSince = simTime;
                if (simTime - stuckSince > restartWhenStuckFor && !restartPending)
                {
                    Debug.Log($"SIH live: the car has not moved for {restartWhenStuckFor:0} s; starting a new world.", this);
                    restartPending = true;
                }
            }
            // Never across a step still being computed: its reply would land
            // in the new world.
            if (restartPending && !awaiting)
            {
                restartPending = false;
                stuckSince = -1f;
                Restart();
            }
        }

        void Handle(Msg m)
        {
            if (m.error != null)
            {
                connected = false; running = false; awaiting = false;
                status = "STACK OFFLINE (" + m.error + ") - waiting for it to reconnect";
                Debug.LogWarning("SIH live: " + status, this);
                return;
            }
            string type = Json.S(m.json, "type");
            switch (type)
            {
                case "ready":
                    connected = true;
                    octaveVersion = Json.S(m.json, "interpreter");
                    status = "connected to " + octaveVersion;
                    SendStart();
                    break;
                case "hello":
                    OnHello(m);
                    break;
                case "act":
                    OnAct(m);
                    break;
                case "error":
                    awaiting = false;
                    paused = true;
                    status = $"STACK ERROR in {Json.S(m.json, "where")}: {Json.S(m.json, "message")} ({Json.S(m.json, "at")}) - paused";
                    Debug.LogError("SIH live: " + status, this);
                    break;
            }
        }

        void SendStart()
        {
            var msg = new Dictionary<string, object> { ["type"] = "start", ["scenario"] = scenario, ["think"] = think };
            if (randomTraffic) msg["layout_seed"] = (double)WorldSeed;
            else if (seed != 0) msg["seed"] = (double)seed;
            if (target == null)
            {
                var g = GameObject.Find("Goal");
                if (g != null) target = g.transform;
            }
            if (target != null)
                msg["goal"] = new List<object> { (double)target.position.x, (double)target.position.z };
            msg["sensors"] = new Dictionary<string, object>
            {
                ["camera"] = SourceName(sensors != null ? sensors.cameraSource : SihSensorRig.Source.Octave),
                ["radar"] = SourceName(sensors != null ? sensors.radarSource : SihSensorRig.Source.Octave),
                ["lidar"] = SourceName(sensors != null ? sensors.lidarSource : SihSensorRig.Source.Octave),
            };
            if (sensors != null && sensors.AnyUnity)
                msg["map"] = new Dictionary<string, object> { ["static"] = sensors.StaticMap() };
            Send(msg, null);
            status = "starting " + scenario;
            running = false;
        }

        static string SourceName(SihSensorRig.Source s) =>
            s == SihSensorRig.Source.Unity ? "unity" : s == SihSensorRig.Source.Off ? "off" : "octave";

        void OnHello(Msg m)
        {
            Run = RunData.FromHeader(Json.Get(m.json, "run"));
            stepDt = Json.D(Json.Get(Json.Get(m.json, "run"), "meta"), "dt", 0.05);
            var e = Doubles(m.blob, 0, 6);
            if (plant != null)
            {
                if (plant.useStackParameters)
                {
                    var p = Json.Get(m.json, "plant");
                    plant.wheelbase = Json.D(p, "wheelbase", plant.wheelbase);
                    plant.aMax = Json.D(p, "a_max", plant.aMax);
                    plant.aEmergency = Json.D(p, "a_emergency", plant.aEmergency);
                    plant.deltaMax = Json.D(p, "delta_max", plant.deltaMax);
                    plant.deltaRate = Json.D(p, "delta_rate", plant.deltaRate);
                    plant.vReverseMax = Json.D(p, "v_reverse_max", plant.vReverseMax);
                }
                plant.SetState(e);
                plant.aCmd = 0; plant.deltaCmd = 0;
            }
            step = 0; simTime = 0f; displayTime = 0f;
            done = false; reached = false; collided = false; doneAt = -1f;
            awaiting = false; plantError = 0;
            pending.Clear(); myDrops.Clear(); dropped = 0;
            senseAgents = Json.Get(m.json, "sense_agents");
            if (sensors != null) sensors.ApplySpec(Json.Get(m.json, "sensor_spec"), Run.rearToCentre);
            var src = Json.Get(m.json, "sensors");
            perception = ""; result = "";
            running = true;
            string via = Json.S(m.json, "bridge");
            status = $"LIVE {Run.name}, world {WorldSeed}{(randomTraffic ? "" : " (scripted traffic)")}: the stack is driving" +
                     (string.IsNullOrEmpty(via) ? "" : $", via {via}") +
                     $" (camera {Json.S(src, "camera")}, radar {Json.S(src, "radar")}, lidar {Json.S(src, "lidar")})";
        }

        void SendTick()
        {
            var msg = new Dictionary<string, object> { ["type"] = "tick", ["k"] = (double)(step + 1) };
            if (pending.Count > 0) { msg["events"] = new List<object>(pending); pending.Clear(); }
            var s = plant != null ? plant.State : new double[6];
            byte[] raw = null;
            if (sensors != null && sensors.AnyUnity)
            {
                // What the sensors see of the world at this step, before the stack runs it.
                raw = sensors.Capture(s, senseAgents, out int nl, out int nr, out int nc);
                msg["n_lidar"] = (double)nl; msg["n_radar"] = (double)nr; msg["n_camera"] = (double)nc;
            }
            var blob = new byte[48 + (raw?.Length ?? 0)];
            for (int i = 0; i < 6; i++) BitConverter.GetBytes(s[i]).CopyTo(blob, 8 * i);
            raw?.CopyTo(blob, 48);
            Send(msg, blob);
            awaiting = true;
            stepOnce = false;
        }

        void OnAct(Msg m)
        {
            awaiting = false;
            var j = m.json;
            step = (int)Json.D(j, "k");
            float t = (float)Json.D(j, "t");
            // [a_cmd delta_cmd] then Octave's own next ego state.
            var b = Doubles(m.blob, 0, 8);
            if (plant != null)
            {
                plant.Step(b[0], b[1], stepDt, (int)Json.D(j, "gear", 1));
                double err = 0;
                var s = plant.State;
                for (int i = 0; i < 6; i++)
                {
                    double d = i == 2 ? SihEgoPlant.WrapPi(s[i] - b[2 + i]) : s[i] - b[2 + i];
                    err = Math.Max(err, Math.Abs(d));
                }
                plantError = Math.Max(plantError, err);
                Run.AppendStep(t, (float)plant.x, (float)plant.y, (float)plant.psi, (float)plant.v,
                               (float)plant.a, (float)plant.delta, Json.Get(j, "series"));
            }
            else
            {
                Run.AppendStep(t, (float)b[2], (float)b[3], (float)b[4], (float)b[5], (float)b[7], (float)b[6], Json.Get(j, "series"));
            }
            var f = Json.Get(j, "frame");
            if (f != null) Run.AppendFrame(f);
            simTime = t;

            var tm = Json.F(j, "timing");
            if (tm.Length > 0) stepTiming = tm;
            senseAgents = Json.Get(j, "sense_agents");
            var pc = Json.Get(j, "percep");
            if (pc != null && sensors != null && sensors.AnyUnity)
                perception = $"lidar {Json.D(pc, "points"):0} returns, {Json.D(pc, "kept"):0} kept, {Json.D(pc, "clusters"):0} clusters, " +
                             $"{Json.D(pc, "objects"):0} objects; radar {Json.D(pc, "radar"):0}; camera {Json.D(pc, "camera"):0}";
            foreach (var id in Json.F(j, "spawned")) myDrops.Add(Mathf.RoundToInt(id));
            reached = Json.D(j, "reached") > 0.5;
            collided = Json.D(j, "collided") > 0.5;
            Run.reached = reached; Run.collided = collided;
            if (Json.D(j, "done") > 0.5 && !done)
            {
                done = true;
                Run.tReached = t;
                status = reached ? $"goal reached at {t:0.0} s" + (collided ? " (with a collision)" : "")
                                 : $"time up at {t:0.0} s";
                var mt = Json.Get(j, "metrics");
                if (mt != null)
                {
                    result = $"{Run.name}: reached {Json.D(mt, "reached"):0} collided {Json.D(mt, "collided"):0} " +
                             $"t {Json.D(mt, "t_end"):0.0} s, min clearance {Json.D(mt, "min_clear"):0.00} m, " +
                             $"latency p95 {Json.D(mt, "latency_p95_ms"):0.0} ms, jerk rms {Json.D(mt, "jerk_lon_rms"):0.00}, " +
                             $"max curvature {Json.D(mt, "curv_max"):0.000}, mean speed {Json.D(mt, "v_mean"):0.00} m/s";
                    Debug.Log("SIH live result: " + result, this);
                }
            }
        }

        public static double[] Doubles(byte[] b, int start, int n)
        {
            var r = new double[n];
            for (int i = 0; i < n && 8 * (start + i) + 8 <= b.Length; i++) r[i] = BitConverter.ToDouble(b, 8 * (start + i));
            return r;
        }

        // ---- controls (keys on the replay forward here in live mode) ---------

        public void TogglePause() => paused = !paused;

        /// One 50 ms step while paused.
        public void StepOnce()
        {
            paused = true;
            stepOnce = true;
            displayTime = Mathf.Max(displayTime, simTime);
        }

        /// Start the scenario again from the beginning.
        public void Restart()
        {
            if (!connected) return;
            if (seed == 0)
            {
                // A new world: new traffic and new scenery, one seed for both.
                worldSeed = 0;
                if (scenery != null && scenery.randomizeOnPlay) scenery.Generate(WorldSeed);
            }
            Run = null;
            running = false;
            awaiting = false;
            SendStart();
        }

        void HandleKeys()
        {
            var kb = Keyboard.current;
            if (kb == null || Run == null || !running) return;
            if (newWorldKey != Key.None && kb[newWorldKey].wasPressedThisFrame) { restartPending = true; return; }
            foreach (var d in drops)
                if (d != null && d.key != Key.None && kb[d.key].wasPressedThisFrame) Drop(d);
            if (removeLastDrop != Key.None && kb[removeLastDrop].wasPressedThisFrame && myDrops.Count > 0)
            {
                int id = myDrops[myDrops.Count - 1];
                myDrops.RemoveAt(myDrops.Count - 1);
                pending.Add(new Dictionary<string, object> { ["kind"] = "remove", ["id"] = (double)id });
            }
        }

        /// Drop a road user on the ground under the mouse pointer.
        public void Drop(DropPreset d)
        {
            var cam = pointerCamera != null ? pointerCamera : Camera.main;
            var mouse = Mouse.current;
            if (cam == null || mouse == null) return;
            var ray = cam.ScreenPointToRay(mouse.position.ReadValue());
            if (Mathf.Abs(ray.direction.y) < 1e-4f) return;
            float s = -ray.origin.y / ray.direction.y;
            if (s <= 0f) return;
            var p = ray.origin + ray.direction * s;
            DropAt(d, p.x, p.z);      // Frames: Unity (x, h, y) is world (x, y)
        }

        /// Drop a road user at world (x, y), heading set from the road there.
        public void DropAt(DropPreset d, float x, float y)
        {
            if (Run == null || !running) return;
            // Heading from the road at the nearest centreline point.
            int best = 0; float bd = float.MaxValue;
            for (int i = 0; i < Run.roadX.Length; i++)
            {
                float dx = Run.roadX[i] - x, dy = Run.roadY[i] - y, dd = dx * dx + dy * dy;
                if (dd < bd) { bd = dd; best = i; }
            }
            float psi = (Run.roadPsi.Length > best ? Run.roadPsi[best] : 0f) + d.headingFromRoad * Mathf.Deg2Rad;
            pending.Add(new Dictionary<string, object>
            {
                ["kind"] = "spawn", ["class"] = d.agentClass, ["mode"] = d.mode,
                ["x"] = (double)x, ["y"] = (double)y, ["psi"] = (double)psi, ["v"] = (double)d.speed,
            });
            dropped++;
            status = $"dropped a {d.name} at ({x:0.0}, {y:0.0})";
        }
    }

    /// Just enough JSON writing for the messages to the stack.
    public static class JsonWrite
    {
        public static string Write(object v)
        {
            var sb = new StringBuilder();
            W(sb, v);
            return sb.ToString();
        }

        static void W(StringBuilder sb, object v)
        {
            switch (v)
            {
                case null: sb.Append("null"); break;
                case string s:
                    sb.Append('"');
                    foreach (char c in s)
                    {
                        if (c == '"' || c == '\\') sb.Append('\\').Append(c);
                        else if (c < 32) sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else sb.Append(c);
                    }
                    sb.Append('"');
                    break;
                case bool b: sb.Append(b ? "true" : "false"); break;
                case double d: sb.Append(d.ToString("R", CultureInfo.InvariantCulture)); break;
                case float f: sb.Append(((double)f).ToString("R", CultureInfo.InvariantCulture)); break;
                case int i: sb.Append(i.ToString(CultureInfo.InvariantCulture)); break;
                case Dictionary<string, object> o:
                    sb.Append('{');
                    bool first = true;
                    foreach (var kv in o)
                    {
                        if (!first) sb.Append(',');
                        first = false;
                        W(sb, kv.Key);
                        sb.Append(':');
                        W(sb, kv.Value);
                    }
                    sb.Append('}');
                    break;
                case System.Collections.IEnumerable e:
                    sb.Append('[');
                    bool f1 = true;
                    foreach (var x in e)
                    {
                        if (!f1) sb.Append(',');
                        f1 = false;
                        W(sb, x);
                    }
                    sb.Append(']');
                    break;
                default: sb.Append(Convert.ToString(v, CultureInfo.InvariantCulture)); break;
            }
        }
    }
}
