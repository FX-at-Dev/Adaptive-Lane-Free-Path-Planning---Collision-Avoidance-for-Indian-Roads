// Builds one scene per scenario, entirely in the editor. Everything the demo
// shows is a saved object in the scene: the road, the scenery, the ego, one
// object per road user, the overlay layers, the LiDAR, the camera, the lights
// and the HUD. Nothing is created in Play; open a scene and move, recolour,
// swap or delete anything.
//
//   SIH > Create Scenes                     the five scenes in Assets/Scenes
//   SIH > Rebuild Prefabs and Materials     reset the models and materials
//   SIH > Rebake Timelines                  redraw the footer timeline images
//
// Prefabs and materials that already exist are reused as they are, so edits
// to them survive Create Scenes; only the Rebuild menu resets them.

using System.Collections.Generic;
using System.IO;
using System.Linq;
using Sih;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace SihEditor
{
    public static class SihSceneBuilder
    {
        const string RunDir = "Assets/SIH/Runs";
        const string AgentDir = "Assets/SIH/Prefabs/Agents";
        const string SceneryDir = "Assets/SIH/Prefabs/Scenery";
        const string GenDir = "Assets/SIH/Generated";
        const string SettingsDir = "Assets/SIH/Settings";
        const string SceneDir = "Assets/Scenes";

        static readonly string[] Scenarios =
            { "village_road", "urban_intersection", "highway_merge", "market", "cattle_crossing" };

        const float EgoL0 = 4.0f, EgoW0 = 1.8f;

        // ---- menus -----------------------------------------------------------

        [MenuItem("SIH/Create Scenes", false, 0)]
        public static void CreateScenesMenu() => CreateScenes(!Application.isBatchMode);

        [MenuItem("SIH/Rebuild Prefabs and Materials", false, 20)]
        public static void RebuildMenu()
        {
            if (!EditorUtility.DisplayDialog("Rebuild prefabs and materials",
                    "Reset every SIH prefab and material to its generated default? Edits made to them will be lost. Scenes keep their references.",
                    "Rebuild", "Cancel")) return;
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            var setup = EditorSceneManager.GetSceneManagerSetup();
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            try
            {
                SihShapes.overwrite = true;
                SihShapes.ClearCache();
                Prefabs(true);
                AssetDatabase.SaveAssets();
            }
            finally { SihShapes.overwrite = false; }
            if (setup.Length > 0) EditorSceneManager.RestoreSceneManagerSetup(setup);
        }

        /// Re-bake the timeline picture of every open scene from the settings
        /// on its SihTimeline.
        [MenuItem("SIH/Rebake Timelines", false, 21)]
        public static void RebakeMenu()
        {
            int n = 0;
            foreach (var tl in Object.FindObjectsByType<SihTimeline>(FindObjectsInactive.Include))
                if (BakeTimeline(tl)) n++;
            AssetDatabase.SaveAssets();
            Debug.Log($"SIH: baked {n} timeline picture(s) in the open scene(s)");
        }

        /// Bake one timeline's picture from its settings and its replay's run.
        public static bool BakeTimeline(SihTimeline tl)
        {
            if (tl == null || tl.replay == null || tl.replay.runFile == null)
            {
                Debug.LogWarning("SIH: the timeline needs a Replay with a Run File to bake its picture", tl);
                return false;
            }
            var run = RunData.Parse(tl.replay.runFile.text);
            string path = tl.band != null && tl.band.texture != null ? AssetDatabase.GetAssetPath(tl.band.texture) : "";
            if (string.IsNullOrEmpty(path) || !path.EndsWith(".png"))
                path = $"{GenDir}/{tl.gameObject.scene.name}/Timeline.png";
            var tex = BakeTimeline(run, path, tl);
            if (tl.band != null && tl.band.texture != tex)
            {
                Undo.RecordObject(tl.band, "Bake timeline");
                tl.band.texture = tex;
                EditorSceneManager.MarkSceneDirty(tl.gameObject.scene);
            }
            return true;
        }

        /// Adds what the live closed loop needs to the scenario scenes already
        /// in Assets/Scenes, keeping every edit made to them: the Live Link,
        /// the ego's plant, spare road users for judge drops, the LIVE badge
        /// and status line on the HUD (V2), and the ego's sensors with the
        /// HUD's camera panel (V3). Parts a scene already has are left alone.
        [MenuItem("SIH/Add Live Link to Scenes", false, 22)]
        public static void AddLiveMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            if (!EditorUtility.DisplayDialog("Add Live Link to Scenes",
                    "Adds the Live Link, the ego plant and sensors, spare road users and the live HUD parts to each scenario scene " +
                    "in Assets/Scenes that does not have them yet, and saves the scenes. Nothing you changed is replaced.",
                    "Add", "Cancel"))
                return;
            var setup = EditorSceneManager.GetSceneManagerSetup();
            var done = new List<string>();
            foreach (var name in Scenarios)
            {
                string path = $"{SceneDir}/{Title(name)}.unity";
                if (!File.Exists(path)) continue;
                var scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
                if (AddLive(name))
                {
                    EditorSceneManager.SaveScene(scene);
                    done.Add(Title(name));
                }
            }
            if (setup.Length > 0) EditorSceneManager.RestoreSceneManagerSetup(setup);
            string msg = done.Count > 0 ? "Added to: " + string.Join(", ", done) : "Every scene already has it.";
            Debug.Log("SIH: " + msg);
            EditorUtility.DisplayDialog("Add Live Link to Scenes", msg, "OK");
        }

        /// The V2 parts, into the open scene; false if it already had them all.
        static bool AddLive(string name)
        {
            var replay = Object.FindFirstObjectByType<SihReplay>(FindObjectsInactive.Include);
            if (replay == null) return false;
            var run = replay.runFile != null ? RunData.Parse(replay.runFile.text) : new RunData();
            bool changed = false;

            SihEgoPlant plant = null;
            if (replay.ego != null)
            {
                plant = replay.ego.GetComponent<SihEgoPlant>();
                if (plant == null)
                {
                    plant = replay.ego.gameObject.AddComponent<SihEgoPlant>();
                    plant.wheelbase = run.wheelbase;
                    changed = true;
                }
            }

            if ((replay.spares == null || replay.spares.Length == 0) && run.roadX != null)
            {
                replay.spares = Spares(run);
                changed = true;
            }

            if (replay.liveLink == null)
            {
                var link = Object.FindFirstObjectByType<SihLiveLink>(FindObjectsInactive.Include);
                if (link == null)
                {
                    link = new GameObject("Live Link").AddComponent<SihLiveLink>();
                    link.scenario = name;
                    link.octaveExe = FindOctave();
                    link.repoRoot = RepoRoot();
                }
                if (link.plant == null) link.plant = plant;
                if (link.pointerCamera == null && replay.cameraRig != null) link.pointerCamera = replay.cameraRig.Cam;
                replay.liveLink = link;
                changed = true;
            }

            var liveLink = replay.liveLink;
            // The Goal flag is the target the car stops at: move it to move the stop.
            if (liveLink != null && liveLink.target == null)
            {
                var goal = GameObject.Find("World/Goal") ?? GameObject.Find("Goal");
                if (goal != null) { liveLink.target = goal.transform; EditorUtility.SetDirty(liveLink); changed = true; }
            }
            if (liveLink != null && liveLink.sensors == null && replay.ego != null)
            {
                var rig = Object.FindFirstObjectByType<SihSensorRig>(FindObjectsInactive.Include) ?? SensorRig(replay.ego, run);
                rig.replay = replay;
                var scenery = GameObject.Find("World/Scenery");
                if ((rig.staticStructure == null || rig.staticStructure.Length == 0) && scenery != null)
                    rig.staticStructure = new[] { scenery.transform };
                liveLink.sensors = rig;
                if (replay.hud != null && replay.hud.GetComponentInChildren<SihCameraPanel>(true) == null)
                    CameraPanel(replay.hud, rig, liveLink);
                EditorUtility.SetDirty(liveLink);
                changed = true;
            }

            // V4: the catalog, generated scenery, generated road users, Car View.
            if (replay.registry == null)
            {
                if (replay.ego != null) EgoLayer(replay.ego.gameObject);
                var sr = replay.liveLink != null ? replay.liveLink.sensors : null;
                if (sr != null && sr.sensorCamera != null) sr.sensorCamera.cullingMask &= ~(1 << EgoLayerIndex);
                var catalog = Catalog();
                var env = Object.FindFirstObjectByType<SihSceneryGenerator>(FindObjectsInactive.Include);
                if (env == null)
                {
                    env = new GameObject("Environment").AddComponent<SihSceneryGenerator>();
                    // The scenery already in the scene is kept, as it is, for
                    // editing; Play makes a new random layout instead of it.
                    var old = GameObject.Find("World/Scenery");
                    if (old != null)
                    {
                        var kept = new GameObject("Kept").transform;
                        kept.SetParent(env.transform, false);
                        old.transform.SetParent(kept, true);
                    }
                }
                env.catalog = catalog;
                env.replay = replay;
                env.liveLink = replay.liveLink;
                var registry = new GameObject("Road Users (generated)").AddComponent<SihAgentRegistry>();
                registry.catalog = catalog;
                replay.registry = registry;
                replay.scenery = env;
                if (replay.liveLink != null)
                {
                    replay.liveLink.scenery = env;
                    if (replay.liveLink.sensors != null)
                        replay.liveLink.sensors.staticStructure = new[] { env.transform };
                }
                CarView(replay.hud, replay.ego, replay.liveLink != null ? replay.liveLink.sensors : null, replay,
                        replay.cameraRig != null ? replay.cameraRig.Cam : Camera.main);
                changed = true;
            }

            var hud = replay.hud;
            if (hud != null && hud.liveBadge == null)
            {
                var sp = hud.scenarioTitle != null ? hud.scenarioTitle.rectTransform.parent as RectTransform : null;
                if (sp != null)
                {
                    uiFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                    uiSprite = AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UISprite.psd");
                    // Make room for the status line, moving the left column
                    // down only where it is still where the generator put it.
                    if (Mathf.Approximately(sp.sizeDelta.y, 84f))
                    {
                        sp.sizeDelta = new Vector2(sp.sizeDelta.x, 104f);
                        foreach (RectTransform r in sp.parent)
                        {
                            if (r == sp || !Mathf.Approximately(r.anchoredPosition.x, 18f)) continue;
                            float y = -r.anchoredPosition.y;
                            bool metric = y >= 114f - 0.5f && y <= 114f + 5 * 74f + 0.5f && Mathf.Approximately((y - 114f) % 74f, 0f);
                            if (metric || Mathf.Approximately(y, 558f))
                                r.anchoredPosition += new Vector2(0f, -18f);
                        }
                    }
                    if (hud.scenarioDesc != null) hud.scenarioDesc.verticalOverflow = VerticalWrapMode.Truncate;
                    LiveHud(hud, sp);
                    changed = true;
                }
            }

            if (changed)
            {
                EditorUtility.SetDirty(replay);
                if (hud != null) EditorUtility.SetDirty(hud);
                EditorSceneManager.MarkSceneDirty(replay.gameObject.scene);
            }
            return changed;
        }

        /// For batch mode: Unity.exe -batchmode -executeMethod SihEditor.SihSceneBuilder.CreateAllBatch
        public static void CreateAllBatch()
        {
            int code = 0;
            try { CreateScenes(false); }
            catch (System.Exception e) { Debug.LogException(e); code = 1; }
            Debug.Log("SIHGEN done " + code);
            if (Application.isBatchMode) EditorApplication.Exit(code);
        }

        // ---- all scenes ------------------------------------------------------

        static void CreateScenes(bool ask)
        {
            if (ask && !EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;

            var runs = CopyRuns();
            if (runs.Count == 0)
            {
                Debug.LogError("SIH: no run files found in results/3d or StreamingAssets/runs");
                return;
            }

            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            SihShapes.ClearCache();
            Prefabs(false);
            var post = PostProfile();
            SihShapes.EnsureDir(SceneDir);

            var paths = Scenarios.Where(runs.ContainsKey).Select(n => $"{SceneDir}/{Title(n)}.unity").ToArray();
            bool replaceAll = !ask;
            string first = null;

            foreach (var name in Scenarios)
            {
                if (!runs.TryGetValue(name, out var file)) continue;
                string path = $"{SceneDir}/{Title(name)}.unity";
                if (File.Exists(path) && !replaceAll)
                {
                    int c = EditorUtility.DisplayDialogComplex("SIH: scene exists",
                        $"{path} already exists. Replace it with a freshly generated scene?\n\nEdits made in that scene will be lost.",
                        "Replace", "Cancel all", "Keep mine");
                    if (c == 1) break;
                    if (c == 2) { first ??= path; continue; }
                }
                EditorUtility.DisplayProgressBar("SIH", $"Building {Title(name)}", 0.5f);
                try { BuildScene(name, file, path, paths, post); }
                finally { EditorUtility.ClearProgressBar(); }
                first ??= path;
            }

            // The scene list Tab cycles through (this is not a build).
            EditorBuildSettings.scenes = paths.Where(File.Exists).Select(p => new EditorBuildSettingsScene(p, true)).ToArray();
            AssetDatabase.SaveAssets();

            string open = File.Exists(paths[0]) ? paths[0] : first;
            if (open != null && !Application.isBatchMode) EditorSceneManager.OpenScene(open, OpenSceneMode.Single);
            Debug.Log($"SIH: scenes ready in {SceneDir}");
        }

        static string Title(string s) =>
            string.Join(" ", s.Split('_').Select(w => w.Length == 0 ? w : char.ToUpper(w[0]) + w.Substring(1)));

        // ---- runs ------------------------------------------------------------

        /// Copy the exported runs into the project as text assets.
        static Dictionary<string, TextAsset> CopyRuns()
        {
            SihShapes.EnsureDir(RunDir);
            string results = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "results", "3d"));
            string streaming = Path.Combine(Application.dataPath, "StreamingAssets", "runs");
            var found = new Dictionary<string, TextAsset>();
            foreach (var name in Scenarios)
            {
                string dst = $"{RunDir}/{name}.json";
                string src = new[] { results, streaming }.Select(d => Path.Combine(d, name + ".json")).FirstOrDefault(File.Exists);
                if (src != null && (!File.Exists(dst) || File.GetLastWriteTimeUtc(src) > File.GetLastWriteTimeUtc(dst)))
                {
                    File.Copy(src, dst, true);
                    AssetDatabase.ImportAsset(dst);
                }
                var ta = AssetDatabase.LoadAssetAtPath<TextAsset>(dst);
                if (ta != null) found[name] = ta;
            }
            return found;
        }

        // ---- prefabs ---------------------------------------------------------

        static GameObject SavePrefab(GameObject g, string dir, string name, bool overwrite)
        {
            SihShapes.EnsureDir(dir);
            string path = $"{dir}/{name}.prefab";
            var existing = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            if (existing != null && !overwrite) { Object.DestroyImmediate(g); return existing; }
            g.name = name;
            var p = PrefabUtility.SaveAsPrefabAsset(g, path);
            Object.DestroyImmediate(g);
            return p;
        }

        static GameObject Load(string dir, string name) => AssetDatabase.LoadAssetAtPath<GameObject>($"{dir}/{name}.prefab");

        static bool Has(string dir, string name) => Load(dir, name) != null;

        /// One prefab per road-user class and scenery piece, made if missing.
        static void Prefabs(bool overwrite)
        {
            var nominal = new RunData();
            foreach (var cls in SihShapes.Classes)
            {
                if (!overwrite && Has(AgentDir, Title(cls))) continue;
                var size = nominal.SizeOf(cls);
                var g = SihShapes.Agent(cls, size.x, size.y);
                var a = g.AddComponent<SihAgent>();
                a.agentClass = cls;
                SavePrefab(g, AgentDir, Title(cls), overwrite);
            }
            if (overwrite || !Has(AgentDir, "Ego")) SavePrefab(SihShapes.Ego(EgoL0, EgoW0), AgentDir, "Ego", overwrite);

            var walls = SihShapes.WallColors;
            var roofs = SihShapes.RoofColors;
            var leaves = SihShapes.LeafColors;
            for (int i = 0; i < walls.Length; i++)
                if (overwrite || !Has(SceneryDir, $"House {i}"))
                    SavePrefab(SihShapes.House(walls[i], roofs[i % roofs.Length]), SceneryDir, $"House {i}", overwrite);
            for (int i = 0; i < leaves.Length; i++)
                if (overwrite || !Has(SceneryDir, $"Tree {i}"))
                    SavePrefab(SihShapes.Tree(leaves[i]), SceneryDir, $"Tree {i}", overwrite);
            for (int i = 0; i < walls.Length; i++)
                if (overwrite || !Has(SceneryDir, $"Wall {i}"))
                    SavePrefab(SihShapes.Wall(Palette.Mul(walls[i], 0.85f)), SceneryDir, $"Wall {i}", overwrite);
            if (overwrite || !Has(SceneryDir, "Pole")) SavePrefab(SihShapes.Pole(), SceneryDir, "Pole", overwrite);
            for (int i = 0; i < leaves.Length; i++)
                if (overwrite || !Has(SceneryDir, $"Field {i}"))
                    SavePrefab(SihShapes.Field(Palette.Mul(leaves[i], 0.55f)), SceneryDir, $"Field {i}", overwrite);
            if (overwrite || !Has(SceneryDir, "Cone")) SavePrefab(SihShapes.Cone(), SceneryDir, "Cone", overwrite);
            if (overwrite || !Has(SceneryDir, "Barrier")) SavePrefab(SihShapes.Barrier(), SceneryDir, "Barrier", overwrite);
            if (overwrite || !Has(SceneryDir, "Sign")) SavePrefab(SihShapes.Sign(), SceneryDir, "Sign", overwrite);
            if (overwrite || !Has(SceneryDir, "Milestone")) SavePrefab(SihShapes.Milestone(), SceneryDir, "Milestone", overwrite);
        }

        static GameObject Place(GameObject prefab, Transform parent, string name = null)
        {
            var g = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            g.transform.SetParent(parent, false);
            if (name != null) g.name = name;
            return g;
        }

        // ---- settings --------------------------------------------------------

        static VolumeProfile PostProfile()
        {
            SihShapes.EnsureDir(SettingsDir);
            string path = $"{SettingsDir}/SIH Post.asset";
            var p = AssetDatabase.LoadAssetAtPath<VolumeProfile>(path);
            if (p != null) return p;
            p = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(p, path);

            var bloom = p.Add<Bloom>(true);
            bloom.threshold.Override(1f);
            bloom.intensity.Override(0.9f);
            bloom.scatter.Override(0.65f);
            var tm = p.Add<Tonemapping>(true);
            tm.mode.Override(TonemappingMode.Neutral);
            var vig = p.Add<Vignette>(true);
            vig.intensity.Override(0.28f);
            vig.smoothness.Override(0.5f);
            var ca = p.Add<ColorAdjustments>(true);
            ca.postExposure.Override(0.15f);
            ca.contrast.Override(8f);
            ca.saturation.Override(6f);
            foreach (var c in p.components)
            {
                c.hideFlags = HideFlags.HideInInspector | HideFlags.HideInHierarchy;
                AssetDatabase.AddObjectToAsset(c, p);
            }
            EditorUtility.SetDirty(p);
            AssetDatabase.SaveAssets();
            return p;
        }

        static T SaveMesh<T>(T obj, string dir, string file) where T : Object
        {
            SihShapes.EnsureDir(dir);
            string path = $"{dir}/{file}";
            var existing = AssetDatabase.LoadAssetAtPath<T>(path);
            if (existing == null) { AssetDatabase.CreateAsset(obj, path); return obj; }
            EditorUtility.CopySerialized(obj, existing);
            existing.name = Path.GetFileNameWithoutExtension(file);
            Object.DestroyImmediate(obj);
            EditorUtility.SetDirty(existing);
            return existing;
        }

        // ---- one scene -------------------------------------------------------

        static void BuildScene(string name, TextAsset file, string path, string[] allScenes, VolumeProfile post)
        {
            var run = RunData.Parse(file.text);
            run.path = file.name;
            string gen = $"{GenDir}/{Title(name)}";

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // Lighting.
            RenderSettings.skybox = null;
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = Palette.Hex("#6b7d99");
            RenderSettings.ambientEquatorColor = Palette.Hex("#3a4456");
            RenderSettings.ambientGroundColor = Palette.Hex("#1a1f27");
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.Linear;
            RenderSettings.fogColor = Palette.Hex("#0e131b");
            RenderSettings.fogStartDistance = 70f;
            RenderSettings.fogEndDistance = 260f;

            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            var cam = camGo.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Palette.Hex("#0e131b");
            cam.nearClipPlane = 0.2f;
            cam.farClipPlane = 600f;
            cam.allowHDR = true;
            var camData = cam.GetUniversalAdditionalCameraData();
            camData.renderPostProcessing = true;
            camData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            camData.antialiasingQuality = AntialiasingQuality.High;
            camGo.AddComponent<AudioListener>();
            var rig = camGo.AddComponent<CameraRig>();

            var sunGo = new GameObject("Sun");
            var sun = sunGo.AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.color = Palette.Hex("#fff1dc");
            sun.intensity = 1.25f;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.65f;
            sunGo.transform.rotation = Quaternion.Euler(48f, 35f, 0f);
            RenderSettings.sun = sun;

            var volGo = new GameObject("Post Processing");
            var vol = volGo.AddComponent<Volume>();
            vol.isGlobal = true;
            vol.priority = 10;
            vol.sharedProfile = post;

            // World.
            var world = new GameObject("World").transform;
            Ground(run, world);
            Road(run, world, gen);
            Goal(run, world);
            var catalog = Catalog();
            var env = new GameObject("Environment").AddComponent<SihSceneryGenerator>();
            env.catalog = catalog;

            // Ego and road users.
            var ego = Place(Load(AgentDir, "Ego"), null, "Ego");
            if (!Mathf.Approximately(run.egoL, EgoL0) || !Mathf.Approximately(run.egoW, EgoW0))
                ego.transform.localScale = new Vector3(run.egoW / EgoW0, 1f, run.egoL / EgoL0);
            var plant = ego.GetComponent<SihEgoPlant>() ?? ego.AddComponent<SihEgoPlant>();
            plant.wheelbase = run.wheelbase;
            EgoLayer(ego);
            var agents = Agents(run);       // the run file's road users, shown while editing
            var registry = new GameObject("Road Users (generated)").AddComponent<SihAgentRegistry>();
            registry.catalog = catalog;

            // LiDAR.
            var lidarGo = new GameObject("LiDAR");
            var lidar = lidarGo.AddComponent<LidarSim>();
            lidar.ego = ego.transform;
            lidar.mountHeight = run.lidarMount;
            lidar.beams = run.lidarBeams;
            lidar.azimuthBins = run.lidarBins;
            lidar.verticalFovLow = run.lidarVLo;
            lidar.verticalFovHigh = run.lidarVHi;
            lidar.range = run.lidarRange;
            lidar.pointMaterial = SihShapes.Points();

            var think = Think();
            var hud = Hud(run, gen);

            var es = new GameObject("EventSystem");
            es.AddComponent<EventSystem>();
            es.AddComponent<InputSystemUIInputModule>().AssignDefaultActions();

            // V2: the stack in Octave drives the ego live.
            var linkGo = new GameObject("Live Link");
            var link = linkGo.AddComponent<SihLiveLink>();
            link.scenario = name;
            link.plant = plant;
            link.pointerCamera = cam;
            link.octaveExe = FindOctave();
            link.repoRoot = RepoRoot();

            // V3: the ego's sensors feed the stack.
            var sensorRig = SensorRig(ego.transform, run);
            sensorRig.staticStructure = new[] { env.transform };
            link.sensors = sensorRig;
            link.scenery = env;

            // The replay that drives it all.
            var replayGo = new GameObject("Replay");
            replayGo.transform.SetSiblingIndex(0);
            var replay = replayGo.AddComponent<SihReplay>();
            replay.runFile = file;
            replay.startTime = run.T0;
            replay.previewTime = run.T0;
            replay.ego = ego.transform;
            replay.agents = agents;
            replay.cameraRig = rig;
            replay.think = think;
            replay.lidar = lidar;
            replay.hud = hud;
            replay.liveLink = link;
            replay.registry = registry;
            replay.scenery = env;
            sensorRig.replay = replay;
            CameraPanel(hud, sensorRig, link);
            CarView(hud, ego.transform, sensorRig, replay, cam);

            // A layout to look at while editing; Play makes a new one.
            env.replay = replay;
            env.liveLink = link;
            env.Generate(26037);
            replay.scenarioScenes = allScenes;
            if (hud.timeline != null) hud.timeline.replay = replay;
            foreach (var row in hud.layerRows) if (row != null) row.replay = replay;

            replay.PreviewAt(run.T0);
            EditorSceneManager.SaveScene(scene, path);
        }

        // ---- world -----------------------------------------------------------

        static void Ground(RunData run, Transform world)
        {
            var b = new Bounds(Frames.Pos(run.roadX[0], run.roadY[0]), Vector3.zero);
            for (int i = 0; i < run.roadX.Length; i++) b.Encapsulate(Frames.Pos(run.roadX[i], run.roadY[i]));
            b.Expand(new Vector3(160f, 0f, 160f));
            var g = GameObject.CreatePrimitive(PrimitiveType.Plane);   // keeps its collider: the LiDAR sees the ground
            g.name = "Ground";
            g.transform.SetParent(world, false);
            g.transform.position = new Vector3(b.center.x, -0.02f, b.center.z);
            g.transform.localScale = new Vector3(b.size.x / 10f, 1f, b.size.z / 10f);
            g.GetComponent<MeshRenderer>().sharedMaterial = SihShapes.Lit(Palette.Hex("#1a2129"), 0.05f);
        }

        static void Road(RunData run, Transform world, string gen)
        {
            float[] hw = run.roadHw;
            float Hw(int i) => hw[Mathf.Min(i, hw.Length - 1)];
            Band(run, world, gen, "Shoulder", i => Hw(i) + 0.9f, 0f, Palette.Hex("#2a2f33"));
            Band(run, world, gen, "Road", Hw, 0.006f, Palette.Road);

            // Edge lines and a faint dashed centre guide (the road is unmarked
            // in the scenario; the guide is only there for orientation).
            int n = run.roadX.Length;
            var lb = new LineBuilder();
            for (int side = -1; side <= 1; side += 2)
            {
                var pts = new Vector3[n];
                for (int i = 0; i < n; i++)
                {
                    float psi = run.roadPsi[i], h = Hw(i) - 0.15f;
                    pts[i] = Frames.Pos(run.roadX[i] - Mathf.Sin(psi) * h * side, run.roadY[i] + Mathf.Cos(psi) * h * side, 0.016f);
                }
                lb.Strip(pts, 0.12f, Palette.A(Palette.Hex("#b8c2d0"), 0.55f), true);
            }
            LineObject(lb, world, gen, "Edge Lines");

            lb = new LineBuilder();
            float acc = 0f;
            for (int i = 1; i < n; i++)
            {
                var a = Frames.Pos(run.roadX[i - 1], run.roadY[i - 1], 0.015f);
                var b = Frames.Pos(run.roadX[i], run.roadY[i], 0.015f);
                acc += Vector3.Distance(a, b);
                if (Mathf.Repeat(acc, 6f) < 3f) lb.Seg(a, b, 0.12f, Palette.A(Palette.Centre, 0.9f), true);
            }
            LineObject(lb, world, gen, "Centre Dashes");
        }

        static void LineObject(LineBuilder lb, Transform world, string gen, string name)
        {
            var m = new Mesh();
            lb.WriteTo(m);
            m = SaveMesh(m, gen, name + ".asset");
            var g = new GameObject(name);
            g.transform.SetParent(world, false);
            g.AddComponent<MeshFilter>().sharedMesh = m;
            var r = g.AddComponent<MeshRenderer>();
            r.sharedMaterial = SihShapes.Line("Road", 1f, true, 0);
            r.shadowCastingMode = ShadowCastingMode.Off;
            r.receiveShadows = false;
        }

        static void Band(RunData run, Transform world, string gen, string name, System.Func<int, float> width, float h, Color c)
        {
            int n = run.roadX.Length;
            var v = new Vector3[2 * n];
            var uv = new Vector2[2 * n];
            var tri = new int[6 * (n - 1)];
            float s = 0f;
            for (int i = 0; i < n; i++)
            {
                if (i > 0) s += Vector2.Distance(new Vector2(run.roadX[i], run.roadY[i]), new Vector2(run.roadX[i - 1], run.roadY[i - 1]));
                float psi = run.roadPsi[i];
                float nx = -Mathf.Sin(psi), ny = Mathf.Cos(psi), w = width(i);
                v[2 * i] = Frames.Pos(run.roadX[i] + nx * w, run.roadY[i] + ny * w, h);
                v[2 * i + 1] = Frames.Pos(run.roadX[i] - nx * w, run.roadY[i] - ny * w, h);
                uv[2 * i] = new Vector2(0, s);
                uv[2 * i + 1] = new Vector2(1, s);
            }
            for (int i = 0; i < n - 1; i++)
            {
                int k = 6 * i, a = 2 * i;
                tri[k] = a; tri[k + 1] = a + 2; tri[k + 2] = a + 1;
                tri[k + 3] = a + 1; tri[k + 4] = a + 2; tri[k + 5] = a + 3;
            }
            var m = new Mesh { indexFormat = IndexFormat.UInt32 };
            m.vertices = v; m.uv = uv; m.triangles = tri;
            m.RecalculateNormals();
            if (m.normals.Length > 0 && m.normals[0].y < 0)
            {
                for (int i = 0; i < tri.Length; i += 3) { int t = tri[i + 1]; tri[i + 1] = tri[i + 2]; tri[i + 2] = t; }
                m.triangles = tri;
                m.RecalculateNormals();
            }
            m.RecalculateBounds();
            m = SaveMesh(m, gen, name + ".asset");

            var g = new GameObject(name);
            g.transform.SetParent(world, false);
            g.AddComponent<MeshFilter>().sharedMesh = m;
            var r = g.AddComponent<MeshRenderer>();
            r.sharedMaterial = SihShapes.Lit(c, 0.15f);
            r.shadowCastingMode = ShadowCastingMode.Off;
            g.AddComponent<MeshCollider>().sharedMesh = m;
        }

        static void Goal(RunData run, Transform world)
        {
            var g = new GameObject("Goal");
            g.transform.SetParent(world, false);
            g.transform.position = Frames.Pos(run.goal.x, run.goal.y, 0.02f);
            g.transform.rotation = Frames.Rot(run.roadPsi[run.roadPsi.Length - 1] + Mathf.PI / 2);
            SihShapes.MeshPart(g.transform, SihShapes.Disc(), Vector3.zero, new Vector3(3.2f, 1f, 3.2f),
                               SihShapes.Lit(Palette.Goal, 0.2f, 1.6f), "disc");
            SihShapes.Part(g.transform, PrimitiveType.Cylinder, new Vector3(0, 2.2f, 0), new Vector3(0.08f, 2.2f, 0.08f), Palette.Hex("#cbd5e1"), name: "pole");
            SihShapes.Part(g.transform, PrimitiveType.Cube, new Vector3(0.45f, 4.0f, 0), new Vector3(0.9f, 0.55f, 0.03f), Palette.Goal, 0.2f, 2f, "flag");
        }

        // ---- scenery (seeded, so a scenario always looks the same) -----------

        static void Scenery(RunData run, Transform world)
        {
            var sc = new GameObject("Scenery").transform;
            sc.SetParent(world, false);
            var fields = new GameObject("Fields").transform;
            fields.SetParent(sc, false);
            var rng = new System.Random(26037);
            float R(float a, float b) => a + (float)rng.NextDouble() * (b - a);

            int nWall = SihShapes.WallColors.Length, nRoof = SihShapes.RoofColors.Length, nLeaf = SihShapes.LeafColors.Length;
            int n = run.roadX.Length;
            var arc = new float[n];
            for (int i = 1; i < n; i++)
                arc[i] = arc[i - 1] + Vector2.Distance(new Vector2(run.roadX[i], run.roadY[i]), new Vector2(run.roadX[i - 1], run.roadY[i - 1]));
            float total = arc[n - 1];

            var placed = new List<Vector3>();   // x, z, radius
            bool Free(Vector2 p, float r)
            {
                if (DistToRoad(run, p) < r + 0.6f) return false;
                foreach (var q in placed)
                    if ((new Vector2(q.x, q.y) - p).sqrMagnitude < (r + q.z) * (r + q.z)) return false;
                return true;
            }

            for (int side = -1; side <= 1; side += 2)
            {
                float s = R(-20f, 0f);
                while (s < total + 30f)
                {
                    s += R(7f, 16f);
                    Pose(run, arc, s, out var c, out float psi, out float hw);
                    var nrm = new Vector2(-Mathf.Sin(psi), Mathf.Cos(psi)) * side;
                    var fwd = new Vector2(Mathf.Cos(psi), Mathf.Sin(psi));
                    float roll = (float)rng.NextDouble();

                    if (roll < 0.45f)
                    {
                        float w = R(5f, 8f), d = R(4.5f, 7f), h = R(2.8f, 4.2f);
                        var p = c + nrm * (hw + R(3.5f, 6f) + d / 2) + fwd * R(-1f, 1f);
                        float rad = Mathf.Max(w, d) * 0.62f;
                        if (!Free(p, rad)) continue;
                        int wi = rng.Next(nWall);
                        rng.Next(nRoof);   // the roof comes with the house prefab
                        var house = Place(Load(SceneryDir, $"House {wi}"), sc);
                        house.transform.position = Frames.Pos(p.x, p.y);
                        house.transform.rotation = Quaternion.LookRotation(-new Vector3(nrm.x, 0, nrm.y));   // face the road
                        house.transform.localScale = new Vector3(w / SihShapes.HouseW, h / SihShapes.HouseH, d / SihShapes.HouseD);
                        placed.Add(new Vector3(p.x, p.y, rad));

                        if (rng.NextDouble() < 0.5)
                        {
                            var wp = c + nrm * (hw + R(1.6f, 2.4f));
                            if (DistToRoad(run, wp) > 1.2f)
                            {
                                var wall = Place(Load(SceneryDir, $"Wall {rng.Next(nWall)}"), sc);
                                wall.transform.position = Frames.Pos(wp.x, wp.y);
                                wall.transform.rotation = Frames.Rot(psi);
                                wall.transform.localScale = new Vector3(1f, 1f, w * 0.9f / SihShapes.WallLength);
                            }
                        }
                    }
                    else if (roll < 0.9f)
                    {
                        int k = rng.Next(1, 4);
                        for (int j = 0; j < k; j++)
                        {
                            float r = R(1.4f, 2.6f), h = R(4.5f, 8f);
                            var p = c + nrm * (hw + R(2.5f, 9f)) + fwd * R(-4f, 4f);
                            if (!Free(p, r * 0.8f)) continue;
                            var tree = Place(Load(SceneryDir, $"Tree {rng.Next(nLeaf)}"), sc);
                            tree.transform.position = Frames.Pos(p.x, p.y);
                            tree.transform.rotation = Quaternion.Euler(0, R(0, 360), 0);
                            tree.transform.localScale = new Vector3(r / SihShapes.TreeR, h / SihShapes.TreeH, r / SihShapes.TreeR);
                            placed.Add(new Vector3(p.x, p.y, r * 0.8f));
                        }
                    }
                    else
                    {
                        var p = c + nrm * (hw + R(1.2f, 1.8f));
                        if (!Free(p, 0.3f)) continue;
                        var pole = Place(Load(SceneryDir, "Pole"), sc);
                        pole.transform.position = Frames.Pos(p.x, p.y);
                        pole.transform.rotation = Frames.Rot(psi);
                        placed.Add(new Vector3(p.x, p.y, 0.3f));
                    }
                }
            }

            // Fields beyond the village: low flat patches for depth.
            for (int k = 0; k < 40; k++)
            {
                float s = R(-30f, total + 30f);
                int side = rng.NextDouble() < 0.5 ? -1 : 1;
                Pose(run, arc, s, out var c, out float psi, out float hw);
                var nrm = new Vector2(-Mathf.Sin(psi), Mathf.Cos(psi)) * side;
                var p = c + nrm * (hw + R(18f, 60f));
                var size = new Vector3(R(10, 25), 0.02f, R(10, 25));
                int li = rng.Next(nLeaf);
                float shade = Mathf.Round(R(0.45f, 0.65f) * 20f) / 20f;   // a few shades, so a few materials
                var f = SihShapes.Part(fields, PrimitiveType.Cube, Frames.Pos(p.x, p.y, 0.01f), size,
                                       Palette.Mul(SihShapes.LeafColors[li], shade), 0.02f, name: "field");
                f.transform.rotation = Frames.Rot(psi);
                f.GetComponent<MeshRenderer>().shadowCastingMode = ShadowCastingMode.Off;
            }
        }

        /// Pose at arc length s along the centreline, extrapolated past the ends.
        static void Pose(RunData run, float[] arc, float s, out Vector2 c, out float psi, out float hw)
        {
            int n = arc.Length, i = 0;
            while (i < n - 2 && arc[i + 1] < s) i++;
            float u = (s - arc[i]) / Mathf.Max(arc[i + 1] - arc[i], 1e-4f);
            c = Vector2.LerpUnclamped(new Vector2(run.roadX[i], run.roadY[i]), new Vector2(run.roadX[i + 1], run.roadY[i + 1]), u);
            psi = Frames.LerpAngle(run.roadPsi[i], run.roadPsi[i + 1], Mathf.Clamp01(u));
            hw = Mathf.Lerp(run.roadHw[Mathf.Min(i, run.roadHw.Length - 1)], run.roadHw[Mathf.Min(i + 1, run.roadHw.Length - 1)], Mathf.Clamp01(u));
        }

        /// Distance from p to the corridor edge (negative inside).
        static float DistToRoad(RunData run, Vector2 p)
        {
            float best = float.MaxValue;
            for (int i = 0; i < run.roadX.Length - 1; i++)
            {
                var a = new Vector2(run.roadX[i], run.roadY[i]);
                var ab = new Vector2(run.roadX[i + 1], run.roadY[i + 1]) - a;
                float u = Mathf.Clamp01(Vector2.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-6f));
                best = Mathf.Min(best, Vector2.Distance(p, a + ab * u) - run.roadHw[Mathf.Min(i, run.roadHw.Length - 1)]);
            }
            return best;
        }

        // ---- road users ------------------------------------------------------

        static SihAgent[] Agents(RunData run)
        {
            var root = new GameObject("Road Users").transform;
            var nominal = new RunData();
            var list = new List<SihAgent>();
            var done = new HashSet<int>();
            foreach (var f in run.frames)
                for (int k = 0; k < f.agentId.Length; k++)
                {
                    int id = f.agentId[k];
                    if (!done.Add(id)) continue;
                    string cls = f.agentClass != null && k < f.agentClass.Length ? f.agentClass[k] : "car";
                    var prefab = Load(AgentDir, Title(cls)) ?? Load(AgentDir, "Car");
                    var g = Place(prefab, root, $"Agent {id} {cls}");
                    var a = g.GetComponent<SihAgent>() ?? g.AddComponent<SihAgent>();
                    a.agentId = id;
                    a.agentClass = cls;
                    Vector2 want = run.SizeOf(cls), nom = nominal.SizeOf(cls);
                    if (!Mathf.Approximately(want.x, nom.x) || !Mathf.Approximately(want.y, nom.y))
                        g.transform.localScale = new Vector3(want.y / nom.y, 1f, want.x / nom.x);
                    g.transform.SetPositionAndRotation(Frames.Pos(f.agents[3 * k], f.agents[3 * k + 1]), Frames.Rot(f.agents[3 * k + 2]));
                    list.Add(a);
                }
            return list.ToArray();
        }

        /// The ego's sensors (V3): the rig, and a sensor camera on the ego that
        /// renders only when the rig asks, into a texture the HUD shows.
        static SihSensorRig SensorRig(Transform ego, RunData run)
        {
            var go = new GameObject("Sensors");
            var rig = go.AddComponent<SihSensorRig>();
            rig.ego = ego;
            rig.rearToCentre = run.rearToCentre;
            rig.lidarRange = run.lidarRange;
            rig.lidarMount = run.lidarMount;
            rig.beams = run.lidarBeams;
            rig.azimuthBins = run.lidarBins;
            rig.lidarVFovLow = run.lidarVLo;
            rig.lidarVFovHigh = run.lidarVHi;

            var camGo = new GameObject("Sensor Camera");
            camGo.transform.SetParent(ego, false);
            camGo.transform.localPosition = new Vector3(0f, rig.cameraMount, 0f);
            var cam = camGo.AddComponent<Camera>();
            cam.enabled = false;                    // rendered by the rig, not every frame
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Palette.Hex("#0e131b");
            cam.cullingMask = ~((1 << 5) | (1 << EgoLayerIndex));   // not the HUD, not the car's own body
            cam.nearClipPlane = 0.3f;
            cam.farClipPlane = 300f;
            cam.fieldOfView = 2f * Mathf.Atan(Mathf.Tan(rig.cameraFov * 0.5f * Mathf.Deg2Rad) * rig.imageHeight / rig.imageWidth) * Mathf.Rad2Deg;
            var data = cam.GetUniversalAdditionalCameraData();
            data.renderPostProcessing = false;
            data.antialiasing = AntialiasingMode.None;
            rig.sensorCamera = cam;

            string rtPath = $"{GenDir}/Sensor Camera.renderTexture";
            var rt = AssetDatabase.LoadAssetAtPath<RenderTexture>(rtPath);
            if (rt == null)
            {
                SihShapes.EnsureDir(GenDir);
                rt = new RenderTexture(640, 360, 24, RenderTextureFormat.ARGB32) { name = "Sensor Camera" };
                AssetDatabase.CreateAsset(rt, rtPath);
            }
            rig.view = rt;
            return rig;
        }

        /// The HUD panel showing what the camera saw and the boxes it reported.
        static void CameraPanel(SihHud hud, SihSensorRig rig, SihLiveLink link)
        {
            if (hud == null || hud.hudRoot == null) return;
            uiFont ??= Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            uiSprite ??= AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UISprite.psd");
            var root = (RectTransform)hud.hudRoot.transform;
            var panel = Rect("Camera Panel", root, new Vector2(1, 1), new Vector2(1, 1), new Vector2(1, 1), new Vector2(-18, -172), new Vector2(460, 287));
            var cp = panel.gameObject.AddComponent<SihCameraPanel>();
            cp.sensors = rig;
            cp.liveLink = link;
            var content = Fill("Content", panel);
            Img(content, PanelColor, false);
            cp.content = content.gameObject;
            cp.caption = TxtAt("Caption", content, 10, 4, 440, 20, "CAMERA", 11, Palette.Dim, style: FontStyle.Bold);
            var area = TL("Image", content, 0, 28, 460, 259);
            var raw = area.gameObject.AddComponent<RawImage>();
            raw.texture = rig.view;
            raw.raycastTarget = false;
            cp.view = raw;
            cp.boxArea = area;
            var names = rig.classes;
            cp.classColors = new Color[names.Length];
            for (int i = 0; i < names.Length; i++) cp.classColors[i] = Palette.OfClass(names[i]);
            const int N = 16;
            cp.frames = new RectTransform[N];
            cp.labels = new Text[N];
            for (int i = 0; i < N; i++)
            {
                var f = TL($"Box {i + 1}", area, 0, 0, 40, 40);
                foreach (var (n, aMin, aMax, size) in new[]
                {
                    ("Top", new Vector2(0, 1), new Vector2(1, 1), new Vector2(0, 2)),
                    ("Bottom", new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 2)),
                    ("Left", new Vector2(0, 0), new Vector2(0, 1), new Vector2(2, 0)),
                    ("Right", new Vector2(1, 0), new Vector2(1, 1), new Vector2(2, 0)),
                })
                {
                    var e = Rect(n, f, aMin, aMax, new Vector2(0.5f, 0.5f), Vector2.zero, size);
                    Img(e, Palette.Sensor[0], false, false);
                }
                var lab = Rect("Label", f, new Vector2(0, 1), new Vector2(0, 1), new Vector2(0, 0), new Vector2(0, 2), new Vector2(140, 14));
                cp.labels[i] = Txt(lab, "", 10, Palette.Sensor[0], TextAnchor.LowerLeft, FontStyle.Bold);
                cp.labels[i].horizontalOverflow = HorizontalWrapMode.Overflow;
                f.gameObject.SetActive(false);
                cp.frames[i] = f;
            }
            content.gameObject.SetActive(false);
        }

        /// The ego's own model goes on its own layer, so the cameras that look
        /// out of the car (Car View, the sensor camera) do not see the inside
        /// of its roof. The layer is named "Ego" if that slot is free.
        public const int EgoLayerIndex = 3;

        static void EgoLayer(GameObject ego)
        {
            var tm = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            var layers = tm.FindProperty("layers");
            if (layers != null && layers.arraySize > EgoLayerIndex)
            {
                var l = layers.GetArrayElementAtIndex(EgoLayerIndex);
                if (string.IsNullOrEmpty(l.stringValue)) { l.stringValue = "Ego"; tm.ApplyModifiedProperties(); }
            }
            foreach (var t in ego.GetComponentsInChildren<Transform>(true))
                if (t.GetComponent<Camera>() == null) t.gameObject.layer = EgoLayerIndex;
        }

        /// The prefab catalog (V4): every road-user class and scenery kind,
        /// with its prefab variants. Made if missing; entries already there
        /// (your edits) are left alone, missing ones are added.
        static SihCatalog Catalog()
        {
            string path = "Assets/SIH/Catalog.asset";
            var cat = AssetDatabase.LoadAssetAtPath<SihCatalog>(path);
            if (cat == null)
            {
                cat = ScriptableObject.CreateInstance<SihCatalog>();
                AssetDatabase.CreateAsset(cat, path);
            }
            var users = new List<SihCatalog.Entry>(cat.roadUsers ?? new SihCatalog.Entry[0]);
            foreach (var cls in SihShapes.Classes)
                if (!users.Exists(e => e != null && e.key == cls))
                    users.Add(Entry(cls, Load(AgentDir, Title(cls))));
            cat.roadUsers = users.ToArray();
            if (cat.fallbackRoadUser == null) cat.fallbackRoadUser = Load(AgentDir, "Car");

            var kinds = new List<SihCatalog.Entry>(cat.scenery ?? new SihCatalog.Entry[0]);
            void Kind(string key, params GameObject[] prefabs)
            {
                if (!kinds.Exists(e => e != null && e.key == key)) kinds.Add(Entry(key, prefabs));
            }
            var houses = new List<GameObject>(); var trees = new List<GameObject>(); var walls = new List<GameObject>(); var fields = new List<GameObject>();
            for (int i = 0; i < SihShapes.WallColors.Length; i++) { houses.Add(Load(SceneryDir, $"House {i}")); walls.Add(Load(SceneryDir, $"Wall {i}")); }
            for (int i = 0; i < SihShapes.LeafColors.Length; i++) { trees.Add(Load(SceneryDir, $"Tree {i}")); fields.Add(Load(SceneryDir, $"Field {i}")); }
            Kind("house", houses.ToArray());
            Kind("tree", trees.ToArray());
            Kind("wall", walls.ToArray());
            Kind("field", fields.ToArray());
            Kind("pole", Load(SceneryDir, "Pole"));
            Kind("cone", Load(SceneryDir, "Cone"));
            Kind("barrier", Load(SceneryDir, "Barrier"));
            Kind("sign", Load(SceneryDir, "Sign"));
            Kind("milestone", Load(SceneryDir, "Milestone"));
            cat.scenery = kinds.ToArray();
            EditorUtility.SetDirty(cat);
            return cat;
        }

        static SihCatalog.Entry Entry(string key, params GameObject[] prefabs)
        {
            var v = new List<SihCatalog.Variant>();
            foreach (var p in prefabs) if (p != null) v.Add(new SihCatalog.Variant { prefab = p, weight = 1f });
            return new SihCatalog.Entry { key = key, variants = v.ToArray() };
        }

        /// Car View (V4): a camera that sees what the car sees, shown as a
        /// picture-in-picture panel (G swaps it to full screen). A camera
        /// called "Car View" already in the scene is used as it is.
        static void CarView(SihHud hud, Transform ego, SihSensorRig rig, SihReplay replay, Camera main)
        {
            var go = GameObject.Find("Car View");
            Camera cam;
            if (go == null)
            {
                go = new GameObject("Car View");
                cam = go.AddComponent<Camera>();
                cam.nearClipPlane = 0.3f;
                cam.farClipPlane = 400f;
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = Palette.Hex("#0e131b");
                cam.allowHDR = true;
                var data = cam.GetUniversalAdditionalCameraData();
                data.renderPostProcessing = true;
                data.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            }
            else cam = go.GetComponent<Camera>() ?? go.AddComponent<Camera>();
            cam.cullingMask &= ~((1 << 5) | (1 << EgoLayerIndex));   // not the HUD, not the car's own body
            var cv = go.GetComponent<SihCarView>() ?? go.AddComponent<SihCarView>();
            cv.ego = ego;
            cv.sensors = rig;
            cv.replay = replay;
            cv.mainCamera = main;

            string rtPath = $"{GenDir}/Car View.renderTexture";
            var rt = AssetDatabase.LoadAssetAtPath<RenderTexture>(rtPath);
            if (rt == null)
            {
                SihShapes.EnsureDir(GenDir);
                rt = new RenderTexture(960, 540, 24, RenderTextureFormat.ARGB32) { name = "Car View", antiAliasing = 2 };
                AssetDatabase.CreateAsset(rt, rtPath);
            }
            cv.pipTexture = rt;

            if (hud == null || hud.hudRoot == null) return;
            uiFont ??= Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            uiSprite ??= AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UISprite.psd");
            var root = (RectTransform)hud.hudRoot.transform;
            var panel = Rect("Car View Panel", root, new Vector2(0, 0), new Vector2(0, 0), new Vector2(0, 0), new Vector2(286, 92), new Vector2(560, 344));
            Img(panel, PanelColor, false);
            TxtAt("Caption", panel, 10, 4, 540, 20, "CAR VIEW  (G full screen)", 11, Palette.Dim, style: FontStyle.Bold);
            var img = TL("Image", panel, 0, 28, 400, 225).gameObject.AddComponent<RawImage>();
            img.texture = rt;
            img.raycastTarget = false;
            cv.pipImage = img;
            cv.objectList = TxtAt("Objects in view", panel, 408, 30, 148, 300, "", 11, Palette.Ink);
            cv.pipPanel = panel.gameObject;

            // Point and press I: what the stack believes about that object.
            var ins = hud.gameObject.GetComponent<SihInspector>() ?? hud.gameObject.AddComponent<SihInspector>();
            ins.replay = replay;
            ins.mainCamera = main;
            ins.carView = cv;
            var ip = Rect("Inspect Panel", root, new Vector2(1, 1), new Vector2(1, 1), new Vector2(1, 1), new Vector2(-18, -470), new Vector2(300, 150));
            Img(ip, PanelColor, false);
            ins.text = TxtAt("Text", ip, 12, 8, 276, 134, "", 13, Palette.Ink);
            ip.gameObject.SetActive(false);
            ins.panel = ip.gameObject;
        }

        /// Inactive road users that show what a judge drops into a live run.
        static SihAgent[] Spares(RunData run)
        {
            var root = new GameObject("Spare Road Users (live drops)").transform;
            var nominal = new RunData();
            var list = new List<SihAgent>();
            int n = 0;
            foreach (var (cls, count) in SpareSet)
                for (int i = 0; i < count; i++)
                {
                    var prefab = Load(AgentDir, Title(cls)) ?? Load(AgentDir, "Car");
                    var g = Place(prefab, root, $"Spare {Title(cls)} {i + 1}");
                    var a = g.GetComponent<SihAgent>() ?? g.AddComponent<SihAgent>();
                    a.agentId = -1;
                    a.agentClass = cls;
                    Vector2 want = run.SizeOf(cls), nom = nominal.SizeOf(cls);
                    if (!Mathf.Approximately(want.x, nom.x) || !Mathf.Approximately(want.y, nom.y))
                        g.transform.localScale = new Vector3(want.y / nom.y, 1f, want.x / nom.x);
                    // Lined up beside the goal, where they are easy to find and edit.
                    g.transform.position = Frames.Pos(run.goal.x + 12f + 3f * n++, run.goal.y + 12f);
                    g.SetActive(false);
                    list.Add(a);
                }
            return list.ToArray();
        }

        static readonly (string, int)[] SpareSet =
        {
            ("cattle", 3), ("pedestrian", 3), ("pushcart", 2), ("bicycle", 1),
            ("two_wheeler", 1), ("auto", 1), ("car", 1),
        };

        /// octave-cli.exe in the usual install places, newest first.
        static string FindOctave()
        {
            var roots = new List<string>();
            string local = System.Environment.GetEnvironmentVariable("LOCALAPPDATA");
            if (!string.IsNullOrEmpty(local)) roots.Add(Path.Combine(local, "Programs", "GNU Octave"));
            roots.Add(@"C:\Program Files\GNU Octave");
            roots.Add(@"C:\Octave");
            foreach (var r in roots)
            {
                if (!Directory.Exists(r)) continue;
                var dirs = Directory.GetDirectories(r);
                System.Array.Sort(dirs);
                System.Array.Reverse(dirs);
                foreach (var d in dirs)
                {
                    var exe = Path.Combine(d, "mingw64", "bin", "octave-cli.exe");
                    if (File.Exists(exe)) return exe.Replace('\\', '/');
                }
            }
            return "";
        }

        /// The SIH repository: this Unity project lives in <repo>/unity/SIH3D.
        static string RepoRoot()
        {
            var r = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));
            return File.Exists(Path.Combine(r, "startup.m")) ? r.Replace('\\', '/') : "";
        }

        // ---- overlays --------------------------------------------------------

        static ThinkLayers Think()
        {
            var go = new GameObject("Think Layers");
            var think = go.AddComponent<ThinkLayers>();

            LineBatch Layer(string name, float intensity, int queue)
            {
                var g = new GameObject(name);
                g.transform.SetParent(go.transform, false);
                g.AddComponent<MeshFilter>();
                var r = g.AddComponent<MeshRenderer>();
                r.sharedMaterial = SihShapes.Line(name, intensity, true, queue);
                r.shadowCastingMode = ShadowCastingMode.Off;
                r.receiveShadows = false;
                return g.AddComponent<LineBatch>();
            }

            think.occupancy = Layer("6 Occupancy", 1.15f, -2);
            think.corridor = Layer("1 Corridor", 1.15f, 0);
            think.detections = Layer("3 Detections", 1.15f, 2);
            think.trackGround = Layer("4 Track Ground", 1.1f, 2);
            think.tracks = Layer("4 Tracks", 1.15f, 3);
            think.predictions = Layer("5 Predictions", 1.15f, 4);
            think.candidates = Layer("7 Candidates", 1.15f, 6);
            think.path = Layer("8 Selected Path", 2.6f, 7);
            think.lookahead = Layer("9 Lookahead", 2.2f, 8);

            var labels = new GameObject("Track Labels").transform;
            labels.SetParent(go.transform, false);
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            var pool = new TextMesh[24];
            for (int i = 0; i < pool.Length; i++)
            {
                var g = new GameObject($"Label {i}");
                g.transform.SetParent(labels, false);
                var tm = g.AddComponent<TextMesh>();
                tm.font = font;
                tm.fontSize = 48;
                tm.characterSize = 0.08f;
                tm.anchor = TextAnchor.LowerCenter;
                tm.alignment = TextAlignment.Center;
                tm.text = "#0 car\n0.0 m/s";
                var mr = g.GetComponent<MeshRenderer>();
                mr.sharedMaterial = font.material;
                mr.shadowCastingMode = ShadowCastingMode.Off;
                mr.receiveShadows = false;
                g.SetActive(false);
                pool[i] = tm;
            }
            think.labels = pool;
            return think;
        }

        // ---- HUD -------------------------------------------------------------

        static Font uiFont;
        static Sprite uiSprite;
        static readonly Color PanelColor = Palette.A(Palette.Panel, 0.93f);

        static RectTransform Rect(string name, Transform parent, Vector2 aMin, Vector2 aMax, Vector2 pivot, Vector2 pos, Vector2 size)
        {
            var g = new GameObject(name, typeof(RectTransform)) { layer = 5 };
            var r = (RectTransform)g.transform;
            r.SetParent(parent, false);
            r.anchorMin = aMin; r.anchorMax = aMax; r.pivot = pivot;
            r.anchoredPosition = pos; r.sizeDelta = size;
            return r;
        }

        /// Pinned to the parent's top-left; x right, y down, in reference pixels.
        static RectTransform TL(string name, Transform parent, float x, float y, float w, float h) =>
            Rect(name, parent, new Vector2(0, 1), new Vector2(0, 1), new Vector2(0, 1), new Vector2(x, -y), new Vector2(w, h));

        static RectTransform Fill(string name, Transform parent) =>
            Rect(name, parent, Vector2.zero, Vector2.one, new Vector2(0.5f, 0.5f), Vector2.zero, Vector2.zero);

        static Image Img(RectTransform r, Color c, bool raycast = true, bool sliced = true)
        {
            var i = r.gameObject.AddComponent<Image>();
            if (sliced) { i.sprite = uiSprite; i.type = Image.Type.Sliced; }
            i.color = c;
            i.raycastTarget = raycast;
            return i;
        }

        static Text Txt(RectTransform r, string s, int size, Color c, TextAnchor align = TextAnchor.UpperLeft, FontStyle style = FontStyle.Normal)
        {
            var t = r.gameObject.AddComponent<Text>();
            t.font = uiFont;
            t.text = s;
            t.fontSize = size;
            t.color = c;
            t.alignment = align;
            t.fontStyle = style;
            t.supportRichText = true;
            t.raycastTarget = false;
            t.horizontalOverflow = HorizontalWrapMode.Wrap;
            t.verticalOverflow = VerticalWrapMode.Overflow;
            return t;
        }

        static Text TxtAt(string name, Transform parent, float x, float y, float w, float h, string s, int size, Color c,
                          TextAnchor align = TextAnchor.UpperLeft, FontStyle style = FontStyle.Normal) =>
            Txt(TL(name, parent, x, y, w, h), s, size, c, align, style);

        /// The LIVE badge and the live status line, in the scenario panel.
        static void LiveHud(SihHud hud, RectTransform sp)
        {
            var lb = Rect("Live Badge", sp, new Vector2(1, 1), new Vector2(1, 1), new Vector2(1, 1), new Vector2(-12, -12), new Vector2(88, 24));
            Img(lb, Palette.A(Palette.Look, 0.2f), false);
            Txt(Fill("Text", lb), "LIVE", 13, Palette.Look, TextAnchor.MiddleCenter, FontStyle.Bold);
            lb.gameObject.SetActive(false);
            hud.liveBadge = lb.gameObject;
            hud.liveStatus = TxtAt("Live Status", sp, 16, 80, 488, 18, "", 13, Palette.Look);
            hud.liveStatus.gameObject.SetActive(false);
        }

        static SihHud Hud(RunData run, string gen)
        {
            uiFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            uiSprite = AssetDatabase.GetBuiltinExtraResource<Sprite>("UI/Skin/UISprite.psd");

            var go = new GameObject("HUD Canvas", typeof(RectTransform)) { layer = 5 };
            var canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
            scaler.matchWidthOrHeight = 1f;
            go.AddComponent<GraphicRaycaster>();
            var hud = go.AddComponent<SihHud>();
            var ct = go.transform;
            Color ink = Palette.Ink, dim = Palette.Dim;

            var root = Fill("HUD", ct);
            hud.hudRoot = root.gameObject;

            // Scenario, top left.
            var sp = TL("Scenario Panel", root, 18, 18, 520, 104);
            Img(sp, PanelColor);
            hud.scenarioTitle = TxtAt("Title", sp, 16, 8, 390, 32, SihHud.Pretty(run.name), 26, ink, style: FontStyle.Bold);
            hud.scenarioDesc = TxtAt("Description", sp, 16, 40, 488, 20, run.desc, 14, dim);
            hud.scenarioDesc.verticalOverflow = VerticalWrapMode.Truncate;   // one line; the rest would run into the status
            hud.statusLine = TxtAt("Status", sp, 16, 60, 488, 18, "t 0.0 s", 13, dim);
            var badge = Rect("Replay Badge", sp, new Vector2(1, 1), new Vector2(1, 1), new Vector2(1, 1), new Vector2(-12, -12), new Vector2(88, 24));
            Img(badge, Palette.A(Palette.Plan, 0.18f), false);
            Txt(Fill("Text", badge), "REPLAY", 13, Palette.Plan, TextAnchor.MiddleCenter, FontStyle.Bold);
            hud.replayBadge = badge.gameObject;
            LiveHud(hud, sp);

            // State, top right.
            var st = Rect("State Panel", root, new Vector2(1, 1), new Vector2(1, 1), new Vector2(1, 1), new Vector2(-18, -18), new Vector2(460, 140));
            Img(st, PanelColor);
            var chip = TL("State Chip", st, 16, 16, 170, 48);
            hud.stateChip = Img(chip, Palette.State["CRUISE"], false);
            hud.stateText = Txt(Fill("State", chip), "CRUISE", 24, Palette.Bg, TextAnchor.MiddleCenter, FontStyle.Bold);
            hud.vCapText = TxtAt("v_cap", st, 204, 14, 240, 20, "v_cap", 15, ink);
            hud.dMaxText = TxtAt("d_max", st, 204, 36, 240, 20, "d_max", 15, ink);
            hud.riskTolText = TxtAt("risk tol", st, 204, 58, 240, 20, "risk tol", 15, ink);
            hud.reasonText = TxtAt("Reason", st, 16, 76, 428, 56, "", 14, dim);

            // Metrics, left column.
            var metrics = new[] { hud.speed, hud.ttc, hud.tracks, hud.clearance, hud.risk, hud.latency };
            string[] mNames = { "SPEED", "TTC", "TRACKS (CONFIRMED)", "MIN CLEARANCE", "PLAN RISK", "REPLAN LATENCY" };
            for (int i = 0; i < metrics.Length; i++)
            {
                var box = TL(Title(mNames[i].ToLowerInvariant().Replace(' ', '_')), root, 18, 132 + i * 74, 250, 66);
                Img(box, PanelColor);
                metrics[i].label = TxtAt("Label", box, 14, 7, 222, 16, mNames[i], 11, dim, style: FontStyle.Bold);
                metrics[i].value = TxtAt("Value", box, 14, 21, 222, 28, "—", 22, ink, style: FontStyle.Bold);
                metrics[i].sub = TxtAt("Sub", box, 14, 47, 222, 16, "", 11, dim);
            }

            // Planner, below the metrics.
            var pl = TL("Planner Panel", root, 18, 576, 250, 92);
            Img(pl, PanelColor);
            TxtAt("Label", pl, 14, 7, 222, 16, "PLANNER", 11, dim, style: FontStyle.Bold);
            hud.plannerSummary = TxtAt("Summary", pl, 14, 24, 222, 20, "", 14, ink);
            var bar = TL("Bar", pl, 14, 48, 222, 10);
            Img(bar, Palette.Line, false);
            RectTransform Seg(string name, Color c)
            {
                var r = Rect(name, bar, Vector2.zero, new Vector2(0, 1), new Vector2(0, 0.5f), Vector2.zero, Vector2.zero);
                Img(r, c, false, false);
                return r;
            }
            hud.acceptSegment = Seg("Feasible", Palette.Plan);
            hud.rejectSegments = new RectTransform[hud.rejectNames.Length];
            for (int i = 0; i < hud.rejectSegments.Length; i++)
                hud.rejectSegments[i] = Seg("Rejected " + hud.rejectNames[i], i < hud.rejectColors.Length ? hud.rejectColors[i] : Color.grey);
            hud.plannerLegend = TxtAt("Legend", pl, 14, 62, 222, 26, "", 11, dim);

            // Layers, bottom right.
            var lp = Rect("Layers Panel", root, new Vector2(1, 0), new Vector2(1, 0), new Vector2(1, 0), new Vector2(-18, 110), new Vector2(220, 238));
            Img(lp, PanelColor);
            TxtAt("Label", lp, 12, 8, 196, 16, "LAYERS  (0 all)", 11, dim, style: FontStyle.Bold);
            hud.layerRows = new SihLayerRow[LayerSet.Count];
            for (int i = 0; i < LayerSet.Count; i++)
            {
                var row = TL($"Row {i + 1} {LayerSet.Names[i]}", lp, 12, 28 + i * 22, 196, 20);
                Img(row, new Color(0, 0, 0, 0), true, false);   // invisible, but catches clicks
                var lr = row.gameObject.AddComponent<SihLayerRow>();
                lr.layer = i;
                var key = TL("Key", row, 0, 1, 18, 18);
                lr.keyBox = Img(key, lr.keyOn, false);
                lr.keyText = Txt(Fill("Text", key), (i + 1).ToString(), 12, Palette.Bg, TextAnchor.MiddleCenter, FontStyle.Bold);
                lr.nameText = TxtAt("Name", row, 26, 0, 170, 20, LayerSet.Names[i], 14, lr.nameOn, TextAnchor.MiddleLeft);
                hud.layerRows[i] = lr;
            }

            // Footer: timeline and keys (stays when the HUD is hidden).
            var ft = Rect("Footer", ct, new Vector2(0, 0), new Vector2(1, 0), new Vector2(0.5f, 0), new Vector2(0, 18), new Vector2(-36, 56));
            Img(ft, PanelColor);
            var track = Rect("Timeline", ft, new Vector2(0, 1), new Vector2(1, 1), new Vector2(0.5f, 1), new Vector2(0, -6), new Vector2(-24, 26));
            var band = track.gameObject.AddComponent<RawImage>();
            band.raycastTarget = true;
            var tl = track.gameObject.AddComponent<SihTimeline>();
            tl.track = track;
            tl.band = band;
            band.texture = BakeTimeline(run, $"{gen}/Timeline.png", tl);
            var head = Rect("Playhead", track, new Vector2(0, 0), new Vector2(0, 1), new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(3, 6));
            Img(head, ink, false, false);
            tl.playhead = head;
            hud.timeline = tl;
            string pc = ColorUtility.ToHtmlStringRGB(Palette.Plan), lc = ColorUtility.ToHtmlStringRGB(Palette.Look);
            Txt(Rect("Legend", ft, new Vector2(0, 0), new Vector2(0.5f, 0), new Vector2(0, 0), new Vector2(12, 3), new Vector2(-12, 20)),
                $"band: state   <color=#{ColorUtility.ToHtmlStringRGB(ink)}>─</color> speed   <color=#{lc}>─</color> clearance   click or drag to jump",
                12, dim, TextAnchor.MiddleLeft);
            Txt(Rect("Keys", ft, new Vector2(0.5f, 0), new Vector2(1, 0), new Vector2(1, 0), new Vector2(-12, 3), new Vector2(-12, 20)),
                $"<color=#{pc}>Space</color> play   ←/→ step   [ ] speed   C camera   1-9 layers   B/P/K drop   G car view   I inspect   H HUD   Tab scenario   F1 help",
                12, dim, TextAnchor.MiddleRight);

            // Help, centred, hidden until F1.
            var help = Rect("Help Panel", ct, new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), Vector2.zero, new Vector2(560, 600));
            Img(help, Palette.A(Palette.Panel, 0.97f));
            TxtAt("Title", help, 24, 18, 512, 30, "Controls", 22, ink, style: FontStyle.Bold);
            TxtAt("Keys", help, 24, 60, 170, 530,
                "Space\n← / →\nShift + ← / →\nHome / End\n[  /  ]\nC\nV\nDrag / wheel\nMiddle drag\n1 – 9\n0\nB / P / K\nBackspace\nG\nI\nN\nH\nTab / Shift+Tab\nF1\nEsc",
                15, Palette.Plan, style: FontStyle.Bold).lineSpacing = 1.15f;
            TxtAt("Actions", help, 200, 60, 336, 530,
                "play / pause\nstep (live: one 50 ms step)\nstep 2 s\nstart / end (live: restart)\nslower / faster\ncamera: chase, top, orbit\nreset the view\norbit, zoom\npan (orbit view)\ntoggle a layer\nall layers on / off\nlive: drop a cow / pedestrian / cart\nlive: remove the last drop\nCar View full screen\ninspect the object at the pointer\ncamera panel on / off\nhide the HUD\nnext / previous scenario\nthis help\nclose help, then quit",
                15, ink).lineSpacing = 1.15f;
            help.gameObject.SetActive(false);
            hud.helpPanel = help.gameObject;

            return hud;
        }

        // ---- timeline image --------------------------------------------------

        static Texture2D BakeTimeline(RunData run, string path, SihTimeline tl)
        {
            SihShapes.EnsureDir(Path.GetDirectoryName(path).Replace('\\', '/'));
            var px = tl.BakeBand(run);
            int w = Mathf.Max(2, tl.bandWidth), h = Mathf.Max(8, tl.bandHeight);
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            tex.SetPixels32(px);
            tex.Apply();
            File.WriteAllBytes(path, tex.EncodeToPNG());
            Object.DestroyImmediate(tex);
            AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            var imp = (TextureImporter)AssetImporter.GetAtPath(path);
            imp.textureType = TextureImporterType.Default;
            imp.mipmapEnabled = false;
            imp.wrapMode = TextureWrapMode.Clamp;
            imp.filterMode = FilterMode.Bilinear;
            imp.npotScale = TextureImporterNPOTScale.None;
            imp.textureCompression = TextureImporterCompression.Uncompressed;
            imp.alphaIsTransparency = true;
            imp.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }
    }
}
