// Inspector for the replay: a time slider that poses the whole scene at any
// moment of the run while editing, so what Play will show can be seen and
// arranged in the Scene view first.

using System.Collections.Generic;
using Sih;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SihEditor
{
    [CustomEditor(typeof(SihReplay))]
    public class SihReplayEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            var r = (SihReplay)target;
            DrawDefaultInspector();
            if (Application.isPlaying || targets.Length > 1) return;

            EditorGUILayout.Space(8);
            EditorGUILayout.LabelField("Edit-mode preview", EditorStyles.boldLabel);
            if (!r.Bind())
            {
                EditorGUILayout.HelpBox("Assign a Run File to preview the replay.", MessageType.Info);
                return;
            }
            var run = r.Run;
            EditorGUI.BeginChangeCheck();
            float to = EditorGUILayout.Slider("Preview time (s)", r.previewTime, run.T0, run.T1);
            if (EditorGUI.EndChangeCheck()) Preview(r, to);

            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Pose at start time")) Preview(r, r.startTime);
                if (GUILayout.Button("Use preview as start time"))
                {
                    Undo.RecordObject(r, "Set start time");
                    r.startTime = r.previewTime;
                    EditorUtility.SetDirty(r);
                }
            }
            if (GUILayout.Button("Refresh overlays")) r.RefreshOverlays();
            EditorGUILayout.HelpBox(
                "The slider moves the ego, the road users and the camera to that moment, so they can be seen here. " +
                "In Play the replay poses them itself, from Start Time.", MessageType.None);
        }

        static void Preview(SihReplay r, float time)
        {
            var objs = new List<Object> { r };
            if (r.ego != null) objs.Add(r.ego);
            if (r.cameraRig != null) { objs.Add(r.cameraRig.transform); objs.Add(r.cameraRig.Cam); }
            foreach (var a in r.agents)
                if (a != null) { objs.Add(a.transform); objs.Add(a.gameObject); }
            Undo.RecordObjects(objs.ToArray(), "Preview replay");
            r.PreviewAt(time);
            EditorSceneManager.MarkSceneDirty(r.gameObject.scene);
            SceneView.RepaintAll();
        }
    }

    /// The scenery generator's edit-mode controls.
    [CustomEditor(typeof(SihSceneryGenerator))]
    public class SihSceneryGeneratorEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var g = (SihSceneryGenerator)target;
            EditorGUILayout.Space(6);
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Preview layout"))
                {
                    Undo.RegisterFullObjectHierarchyUndo(g.gameObject, "Preview layout");
                    g.Generate(g.seed != 0 ? g.seed : System.Environment.TickCount & 0x7fffffff);
                    EditorSceneManager.MarkSceneDirty(g.gameObject.scene);
                }
                if (GUILayout.Button("Keep this layout"))
                {
                    Undo.RegisterFullObjectHierarchyUndo(g.gameObject, "Keep layout");
                    g.Keep();
                    EditorSceneManager.MarkSceneDirty(g.gameObject.scene);
                }
                if (GUILayout.Button("Clear"))
                {
                    Undo.RegisterFullObjectHierarchyUndo(g.gameObject, "Clear layout");
                    g.Clear();
                    EditorSceneManager.MarkSceneDirty(g.gameObject.scene);
                }
            }
            EditorGUILayout.HelpBox(
                $"Last layout: seed {g.lastSeed}, {g.placed} objects. With Randomize On Play, every Play makes a new layout " +
                "(the Live Link's world seed when it drives live) and hides anything under Kept while playing.", MessageType.None);
        }
    }

    /// The timeline's band picture is a texture asset; this re-bakes it from
    /// the colours and sizes set on the component.
    [CustomEditor(typeof(SihTimeline))]
    public class SihTimelineEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            EditorGUILayout.Space(6);
            if (GUILayout.Button("Bake band picture"))
                foreach (var t in targets)
                    SihSceneBuilder.BakeTimeline((SihTimeline)t);
        }
    }
}
