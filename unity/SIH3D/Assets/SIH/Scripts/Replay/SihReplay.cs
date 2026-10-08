// Shows one run in 3D: a recorded run file (V1 replay), or the run the stack
// is driving right now through the Live Link (V2 live closed loop).
//
// Everything it moves is already in the scene: the ego, one object per road
// user (found by SihAgent.agentId), the overlay layers, the LiDAR and the HUD.
// This component only reads the run and, at each moment of playback, poses
// those objects and redraws the overlays from the planner's snapshot.
// Road users a judge drops into a live run are shown by the spare objects
// listed below. Nothing is created while playing.
//
// In the editor, the inspector's preview slider poses the scene at any time
// of the run without entering Play, so it can be looked at and dressed.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

namespace Sih
{
    [ExecuteAlways]
    [DefaultExecutionOrder(-50)]
    public class SihReplay : MonoBehaviour
    {
        [Header("Run")]
        [Tooltip("A run written by sim/sih_export_run.m with think capture on.")]
        public TextAsset runFile;
        [Tooltip("Replay time to start from, seconds.")]
        public float startTime;
        public bool playOnStart = true;

        [Header("Playback")]
        public float speed = 1f;
        [Tooltip("Speeds the [ and ] keys step through.")]
        public float[] speedSteps = { 0.1f, 0.25f, 0.5f, 1f, 2f, 4f };
        [Tooltip("Go back to the start after holding the end.")]
        public bool loop = true;
        public float endHoldSeconds = 4f;
        public float smallStep = 0.1f, bigStep = 2f;

        [Header("Layers (keys 1-9)")]
        public LayerSet layers = new LayerSet();

        [Header("Scene objects")]
        public Transform ego;
        public SihAgent[] agents = new SihAgent[0];
        public CameraRig cameraRig;
        public ThinkLayers think;
        public LidarSim lidar;
        public SihHud hud;
        [Tooltip("When it is enabled and set to Use Live, Play shows the stack driving live instead of the run file.")]
        public SihLiveLink liveLink;

        [Tooltip("Makes the road users from the prefab catalog as the run mentions them. When set, it replaces Agents and Spares.")]
        public SihAgentRegistry registry;
        [Tooltip("The scenery generator, for the world seed of a replay.")]
        public SihSceneryGenerator scenery;

        [Header("Spare road users (live drops, without a registry)")]
        [Tooltip("Inactive objects that show road users dropped into a live run, matched by class. Add more by duplicating one.")]
        public SihAgent[] spares = new SihAgent[0];

        [Header("Scenarios (Tab / Shift+Tab)")]
        [Tooltip("Scene names or paths, in order. They must be in the Build Profiles scene list.")]
        public string[] scenarioScenes = new string[0];

        [Header("Keys")]
        public Key playPause = Key.Space;
        public Key stepForward = Key.RightArrow, stepBack = Key.LeftArrow;
        [Tooltip("Hold with a step key for the big step (either key works).")]
        public Key bigStepModifier = Key.LeftShift, bigStepModifierAlt = Key.RightShift;
        public Key toStart = Key.Home, toEnd = Key.End;
        public Key slower = Key.LeftBracket, faster = Key.RightBracket;
        public Key cycleCamera = Key.C, resetView = Key.V;
        public Key toggleHud = Key.H, toggleHelp = Key.F1;
        public Key nextScenario = Key.Tab;
        public Key quit = Key.Escape;
        public Key allLayers = Key.Digit0, allLayersAlt = Key.Numpad0;
        public Key[] layerKeys = { Key.Digit1, Key.Digit2, Key.Digit3, Key.Digit4, Key.Digit5, Key.Digit6, Key.Digit7, Key.Digit8, Key.Digit9 };
        public Key[] layerKeysAlt = { Key.Numpad1, Key.Numpad2, Key.Numpad3, Key.Numpad4, Key.Numpad5, Key.Numpad6, Key.Numpad7, Key.Numpad8, Key.Numpad9 };

        [Header("Display")]
        [Tooltip("-1: as fast as vSync allows.")]
        public int targetFrameRate = -1;
        public int vSyncCount = 1;
        [Tooltip("How quickly the fps reading follows the frame time (0..1 per frame).")]
        [Range(0.001f, 1f)] public float fpsSmoothing = 0.05f;

        [Header("Editor preview")]
        [Tooltip("The time the scene is posed at in edit mode.")]
        public float previewTime;

        [Header("Live (read only)")]
        public float t;
        public bool playing;
        public float fps = 60f;
        [Tooltip("Ego speed now, m/s.")]
        public float egoSpeed;
        [Tooltip("Ego heading now, radians (counter-clockwise from east).")]
        public float egoPsi;
        [Tooltip("True while the timeline is being dragged.")]
        public bool scrubbing;

        public RunData Run { get; private set; }

        /// Playing the stack live through the Live Link.
        public bool IsLive => Application.isPlaying && liveLink != null && liveLink.isActiveAndEnabled && liveLink.useLive;
        public bool Scrubbing { get => scrubbing; private set => scrubbing = value; }
        public float EgoV { get => egoSpeed; private set => egoSpeed = value; }

        readonly Dictionary<int, SihAgent> byId = new Dictionary<int, SihAgent>();
        readonly HashSet<int> seen = new HashSet<int>();
        TextAsset boundFile;
        float endHold;

        // ---- run -------------------------------------------------------------

        // Parsed runs, kept across scene loads so switching scenarios and
        // scrubbing in the editor do not re-read the file.
        static readonly Dictionary<TextAsset, RunData> parsed = new Dictionary<TextAsset, RunData>();

        /// A run file, parsed once and kept.
        public static RunData ParseCached(TextAsset file)
        {
            if (file == null) return null;
            if (!parsed.TryGetValue(file, out var run) || run == null)
            {
                run = RunData.Parse(file.text);
                run.path = file.name;
                parsed[file] = run;
            }
            return run;
        }

        /// Every road-user object the scene has for this run.
        public IEnumerable<SihAgent> AllAgents
        {
            get
            {
                if (registry != null) { foreach (var a in registry.All) yield return a; yield break; }
                foreach (var a in agents) if (a != null) yield return a;
                foreach (var a in spares) if (a != null) yield return a;
            }
        }

        int WorldSeed => IsLive ? liveLink.WorldSeed : scenery != null && scenery.lastSeed != 0 ? scenery.lastSeed : 26037;

        /// Parse the run (once per file) and hand it to the scene objects.
        public bool Bind()
        {
            if (IsLive) return Run != null;
            if (runFile == null) { Run = null; return false; }
            if (Run != null && boundFile == runFile) return true;
            if (!parsed.TryGetValue(runFile, out var run) || run == null)
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                run = RunData.Parse(runFile.text);
                run.path = runFile.name;
                parsed[runFile] = run;
                Debug.Log($"SIH: read {runFile.name}: {run.frames.Count} frames, {run.T1:0.0} s, {sw.ElapsedMilliseconds} ms", this);
            }
            BindRun(run);
            boundFile = runFile;
            return true;
        }

        /// Hand a run (from a file, or live) to the scene objects.
        void BindRun(RunData run)
        {
            Run = run;
            boundFile = null;
            byId.Clear();
            if (registry != null && Application.isPlaying)
            {
                // Road users come from the catalog as the run mentions them;
                // edit-mode previews and pre-placed ones stand down.
                registry.Begin(run, WorldSeed);
                foreach (var a in agents) if (a != null && a.gameObject.activeSelf) a.gameObject.SetActive(false);
            }
            else
                foreach (var a in agents)
                    if (a != null) byId[a.agentId] = a;
            foreach (var s in spares)
                if (s != null)
                {
                    s.agentId = -1;
                    if (Application.isPlaying && s.gameObject.activeSelf) s.gameObject.SetActive(false);
                }
            if (think != null) think.Bind(Run);
            if (lidar != null) lidar.Bind(Run);
            if (hud != null) hud.Bind(this);
        }

        /// The scene object for road user id: its own, or a spare already
        /// standing in for it, or a free spare (live drops). Used by the
        /// display and by the sensors, so both use the same object.
        public SihAgent AgentFor(int id, string cls)
        {
            if (byId.TryGetValue(id, out var a) && a != null) return a;
            if (registry != null && Application.isPlaying)
            {
                a = registry.Get(id, cls);
                if (a != null) byId[id] = a;
                return a;
            }
            foreach (var x in agents)
                if (x != null && x.agentId == id) { byId[id] = x; return x; }
            foreach (var s in spares)
                if (s != null && s.agentId == id) { byId[id] = s; return s; }
            return AssignSpare(id, cls);
        }

        /// A road user the scene has no object for (a judge's drop): show it
        /// with a free spare of the same class, or of any class.
        SihAgent AssignSpare(int id, string cls)
        {
            SihAgent pick = null;
            foreach (var s in spares)
                if (s != null && s.agentId < 0 && s.agentClass == cls) { pick = s; break; }
            if (pick == null)
                foreach (var s in spares)
                    if (s != null && s.agentId < 0) { pick = s; break; }
            if (pick == null) return null;
            pick.agentId = id;
            byId[id] = pick;
            return pick;
        }

        // ---- play mode -------------------------------------------------------

        void Awake()
        {
            if (!Application.isPlaying) return;
            Application.targetFrameRate = targetFrameRate;
            QualitySettings.vSyncCount = vSyncCount;
        }

        void Start()
        {
            if (!Application.isPlaying) return;
            if (IsLive)
            {
                Run = null;
                playing = true;
                if (cameraRig != null) cameraRig.ResetView();
                if (hud != null) { hud.Bind(this); hud.Apply(this); }
                return;
            }
            if (!Bind())
            {
                Debug.LogError("SIH: the replay has no run file", this);
                enabled = false;
                return;
            }
            t = Mathf.Clamp(startTime, Run.T0, Run.T1);
            playing = playOnStart;
            endHold = 0f;
            if (cameraRig != null) cameraRig.ResetView();
            Apply(Time.unscaledDeltaTime, true);
        }

        void Update()
        {
            if (!Application.isPlaying) return;
            if (IsLive) { UpdateLive(); return; }
            if (Run == null) return;
            float dt = Time.unscaledDeltaTime;
            if (dt > 1e-5f) fps = Mathf.Lerp(fps, 1f / dt, fpsSmoothing);
            HandleInput();

            if (playing && !Scrubbing)
            {
                if (t < Run.T1) t = Mathf.Min(Run.T1, t + dt * speed);
                else if (loop)
                {
                    // Hold the finish for a moment, then go round again.
                    endHold += dt;
                    if (endHold > endHoldSeconds) { t = Run.T0; endHold = 0f; }
                }
            }
            Apply(dt, false);
        }

        /// Live: show the run the stack is driving, at the Live Link's time.
        void UpdateLive()
        {
            float dt = Time.unscaledDeltaTime;
            if (dt > 1e-5f) fps = Mathf.Lerp(fps, 1f / dt, fpsSmoothing);
            bool fresh = false;
            if (liveLink.Run != Run)
            {
                if (liveLink.Run != null) BindRun(liveLink.Run);
                else Run = null;
                fresh = true;
            }
            HandleInput();
            playing = !liveLink.paused;
            if (Run == null || Run.sT.Length == 0)
            {
                if (hud != null) hud.Apply(this);
                return;
            }
            t = Mathf.Max(liveLink.displayTime, Run.T0);
            Apply(dt, fresh);
        }

        void Apply(float dt, bool snap)
        {
            Pose(t);
            if (cameraRig != null && ego != null)
                cameraRig.Follow(ego.position, egoPsi, dt, snap);
            if (think != null)
            {
                think.Apply(t, layers);
                think.FaceLabels(cameraRig != null ? cameraRig.Cam : Camera.main);
            }
            if (lidar != null) lidar.Tick(t, layers.lidar);
            if (hud != null) hud.Apply(this);
        }

        /// Ego from the 20 Hz series; agents matched by id between the two
        /// planner frames around t and interpolated.
        void Pose(float time)
        {
            if (Run.sT.Length == 0) return;
            Run.EgoAt(time, out float x, out float y, out float psi, out float v);
            egoPsi = psi;
            EgoV = v;
            if (ego != null)
                ego.SetPositionAndRotation(Frames.EgoCentre(x, y, psi, Run.rearToCentre), Frames.Rot(psi));

            int i = Run.FrameAt(time, out float u);
            if (i < 0) return;
            var f0 = Run.frames[i];
            var f1 = Run.frames[Mathf.Min(i + 1, Run.frames.Count - 1)];

            seen.Clear();
            for (int k = 0; k < f0.agentId.Length; k++)
            {
                int id = f0.agentId[k];
                if (!byId.TryGetValue(id, out var a) || a == null)
                {
                    if (!Run.live && registry == null) continue;
                    a = AgentFor(id, k < f0.agentClass.Length ? f0.agentClass[k] : "");
                    if (a == null) continue;
                }
                float ax = f0.agents[3 * k], ay = f0.agents[3 * k + 1], ap = f0.agents[3 * k + 2];
                int j = System.Array.IndexOf(f1.agentId, id);
                if (j >= 0 && f1 != f0)
                {
                    ax = Mathf.Lerp(ax, f1.agents[3 * j], u);
                    ay = Mathf.Lerp(ay, f1.agents[3 * j + 1], u);
                    ap = Frames.LerpAngle(ap, f1.agents[3 * j + 2], u);
                }
                if (!a.gameObject.activeSelf) a.gameObject.SetActive(true);
                a.transform.SetPositionAndRotation(Frames.Pos(ax, ay), Frames.Rot(ap));
                seen.Add(id);
            }
            foreach (var kv in byId)
                if (kv.Value != null && !seen.Contains(kv.Key) && kv.Value.gameObject.activeSelf)
                    kv.Value.gameObject.SetActive(false);
        }

        void HandleInput()
        {
            var kb = Keyboard.current;
            bool live = IsLive;
            if (kb != null)
            {
                bool big = Held(kb, bigStepModifier) || Held(kb, bigStepModifierAlt);
                if (live)
                {
                    // The stack's time only goes forward: pause, single step,
                    // restart and speed go to the Live Link.
                    if (Pressed(kb, playPause)) liveLink.TogglePause();
                    if (Pressed(kb, stepForward)) liveLink.StepOnce();
                    if (Pressed(kb, toStart)) liveLink.Restart();
                    if (Pressed(kb, faster)) liveLink.speed = speed = NextSpeed(+1);
                    if (Pressed(kb, slower)) liveLink.speed = speed = NextSpeed(-1);
                }
                else
                {
                    if (Pressed(kb, playPause)) TogglePlay();
                    float step = big ? bigStep : smallStep;
                    if (Pressed(kb, stepForward)) Seek(t + step);
                    if (Pressed(kb, stepBack)) Seek(t - step);
                    if (Pressed(kb, toStart)) Seek(Run.T0);
                    if (Pressed(kb, toEnd)) Seek(Run.T1);
                    if (Pressed(kb, faster)) speed = NextSpeed(+1);
                    if (Pressed(kb, slower)) speed = NextSpeed(-1);
                }
                if (cameraRig != null && Pressed(kb, cycleCamera)) cameraRig.Cycle();
                if (cameraRig != null && Pressed(kb, resetView)) cameraRig.ResetView();
                if (hud != null && Pressed(kb, toggleHud)) hud.ToggleVisible();
                if (hud != null && Pressed(kb, toggleHelp)) hud.ToggleHelp();
                if (Pressed(kb, nextScenario)) NextScenario(big ? -1 : +1);
                if (Pressed(kb, quit))
                {
                    if (hud != null && hud.HelpOpen) hud.ToggleHelp();
                    else Quit();
                }
                for (int i = 0; i < LayerSet.Count; i++)
                    if ((i < layerKeys.Length && Pressed(kb, layerKeys[i])) || (i < layerKeysAlt.Length && Pressed(kb, layerKeysAlt[i])))
                        ToggleLayer(i);
                if (Pressed(kb, allLayers) || Pressed(kb, allLayersAlt))
                {
                    bool any = layers.Mask() != 0;
                    for (int i = 0; i < LayerSet.Count; i++) layers[i] = !any;
                }
            }

            bool overUi = Scrubbing || (EventSystem.current != null && EventSystem.current.IsPointerOverGameObject());
            if (cameraRig != null) cameraRig.HandleInput(overUi);
        }

        static bool Pressed(Keyboard kb, Key k) => k != Key.None && kb[k].wasPressedThisFrame;
        static bool Held(Keyboard kb, Key k) => k != Key.None && kb[k].isPressed;

        // ---- controls (keys, timeline, layer rows) ---------------------------

        public void TogglePlay()
        {
            if (IsLive) { liveLink.TogglePause(); return; }
            if (Run == null) return;
            if (!playing && t >= Run.T1 - 1e-3f) t = Run.T0;
            playing = !playing;
        }

        public void Seek(float to)
        {
            if (Run == null || IsLive) return;
            t = Mathf.Clamp(to, Run.T0, Run.T1);
            endHold = 0f;
        }

        public void SeekNormalized(float u)
        {
            if (Run != null) Seek(Mathf.Lerp(Run.T0, Run.T1, Mathf.Clamp01(u)));
        }

        public void BeginScrub() => Scrubbing = true;
        public void EndScrub() => Scrubbing = false;

        public void ToggleLayer(int i)
        {
            layers[i] = !layers[i];
#if UNITY_EDITOR
            if (!Application.isPlaying) RequestEditorRefresh();
#endif
        }

        float NextSpeed(int dir)
        {
            if (speedSteps == null || speedSteps.Length == 0) return speed;
            int best = 0;
            for (int i = 1; i < speedSteps.Length; i++)
                if (Mathf.Abs(speedSteps[i] - speed) < Mathf.Abs(speedSteps[best] - speed)) best = i;
            return speedSteps[Mathf.Clamp(best + dir, 0, speedSteps.Length - 1)];
        }

        void NextScenario(int dir)
        {
            if (scenarioScenes == null || scenarioScenes.Length == 0) return;
            string here = SceneManager.GetActiveScene().name;
            int i = System.Array.FindIndex(scenarioScenes, s => System.IO.Path.GetFileNameWithoutExtension(s) == here);
            int n = scenarioScenes.Length;
            int next = ((i < 0 ? 0 : i + dir) % n + n) % n;
            SceneManager.LoadScene(scenarioScenes[next]);
        }

        static void Quit()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }

        // ---- edit mode -------------------------------------------------------

        /// Pose the whole scene at time (edit mode: called by the inspector).
        public void PreviewAt(float time)
        {
            if (!Bind()) return;
            previewTime = Mathf.Clamp(time, Run.T0, Run.T1);
            t = previewTime;
            Pose(t);
            if (cameraRig != null && ego != null && cameraRig.followInEditorPreview)
            {
                cameraRig.ResetView();
                cameraRig.Follow(ego.position, egoPsi, 0f, true);
            }
            RefreshOverlays();
        }

        /// Redraw overlays, LiDAR and HUD without moving anything.
        public void RefreshOverlays()
        {
            if (!Bind()) return;
            t = Mathf.Clamp(Application.isPlaying ? t : previewTime, Run.T0, Run.T1);
            if (!Application.isPlaying)
            {
                Run.EgoAt(t, out _, out _, out egoPsi, out float v);
                EgoV = v;
            }
            if (think != null)
            {
                think.Invalidate();
                think.Apply(t, layers);
                think.FaceLabels(cameraRig != null ? cameraRig.Cam : Camera.main);
            }
            if (lidar != null) lidar.Tick(t, layers.lidar, true);
            if (hud != null) hud.Apply(this);
        }

#if UNITY_EDITOR
        void OnEnable()
        {
            if (!Application.isPlaying) RequestEditorRefresh();
        }

        void OnValidate()
        {
            Run = null;   // rebind: the agent list or scene references may have changed
            if (!Application.isPlaying) RequestEditorRefresh();
        }

        static bool refreshQueued;

        /// Redraw every replay's overlays once the editor is idle.
        public static void RequestEditorRefresh()
        {
            if (refreshQueued) return;
            refreshQueued = true;
            UnityEditor.EditorApplication.delayCall += () =>
            {
                refreshQueued = false;
                if (Application.isPlaying) return;
                foreach (var r in FindObjectsByType<SihReplay>(FindObjectsInactive.Exclude))
                    if (r.isActiveAndEnabled) r.RefreshOverlays();
                UnityEditor.SceneView.RepaintAll();
            };
        }
#endif
    }
}
