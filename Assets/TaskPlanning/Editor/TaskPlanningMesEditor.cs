using System.Linq;
using UnityEditor;
using UnityEngine;

namespace TaskPlanning.Editor
{
    [CustomEditor(typeof(TaskPlanningMes))]
    public sealed class TaskPlanningMesEditor : UnityEditor.Editor
    {
        private static int s_selectedPalletIndex;
        private static int s_selectedWorkstationIndex;
        private static string s_taskId = string.Empty;

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            EditorGUILayout.Space(10);
            DrawSubmitPanel((TaskPlanningMes)target);
        }

        private static void DrawSubmitPanel(TaskPlanningMes mes)
        {
            EditorGUILayout.LabelField("Demo Task", EditorStyles.boldLabel);

            var pallets = Object.FindObjectsByType<PalletMarker>(FindObjectsInactive.Exclude)
                .OrderBy(p => p.PalletId)
                .ToArray();
            var workstations = Object.FindObjectsByType<WorkstationDeliveryPoint>(FindObjectsInactive.Exclude)
                .OrderBy(w => w.WorkstationId)
                .ToArray();

            s_taskId = EditorGUILayout.TextField("Task Id", s_taskId);
            s_selectedPalletIndex = DrawPopup("Kit / Pallet", s_selectedPalletIndex, pallets, p => p.KitId);
            s_selectedWorkstationIndex = DrawPopup("Workstation", s_selectedWorkstationIndex, workstations, w => w.WorkstationId);

            using (new EditorGUI.DisabledScope(!Application.isPlaying || pallets.Length == 0 || workstations.Length == 0))
            {
                if (GUILayout.Button("Submit Task"))
                {
                    var request = new DeliveryTaskRequest(
                        s_taskId,
                        pallets[Mathf.Clamp(s_selectedPalletIndex, 0, pallets.Length - 1)],
                        workstations[Mathf.Clamp(s_selectedWorkstationIndex, 0, workstations.Length - 1)]);
                    mes.SubmitTask(request);
                    s_taskId = string.Empty;
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Enter Play Mode to submit a task to the scheduler.", MessageType.Info);
        }

        private static int DrawPopup<T>(string label, int selectedIndex, T[] values, System.Func<T, string> labelSelector)
            where T : Object
        {
            if (values.Length == 0)
            {
                EditorGUILayout.Popup(label, 0, new[] { "<none in scene>" });
                return 0;
            }

            selectedIndex = Mathf.Clamp(selectedIndex, 0, values.Length - 1);
            var labels = values.Select(labelSelector).ToArray();
            return EditorGUILayout.Popup(label, selectedIndex, labels);
        }
    }
}
