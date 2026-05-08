using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace TaskPlanning
{
    /// <summary>
    /// Centralized Scene view debug drawing for task-planning objects.
    /// </summary>
    public sealed class TaskPlanningDebugGizmos : MonoBehaviour
    {
        [SerializeField] private bool showAmrs = true;
        [SerializeField] private bool showPallets = true;
        [SerializeField] private bool showLoadingPoints = true;
        [SerializeField] private bool showWorkstations = true;
        [SerializeField] private Vector2 amrLabelOffset = new(0f, 0.28f);
        [SerializeField] private Vector2 palletLabelOffset = new(0f, 0.24f);
        [SerializeField] private Vector2 loadingPointLabelOffset = new(0f, 0.24f);
        [SerializeField] private Vector2 workstationLabelOffset = new(0f, 0.24f);

        private void OnDrawGizmos()
        {
            if (showAmrs)
                DrawAmrs();

            if (showPallets)
                DrawPallets();

            if (showLoadingPoints)
                DrawLoadingPoints();

            if (showWorkstations)
                DrawWorkstations();
        }

        private void DrawAmrs()
        {
#if UNITY_EDITOR
            foreach (var amr in FindObjectsByType<TaskPlanningAmr>())
                Handles.Label(amr.transform.position + ToVector3(amrLabelOffset), BuildAmrText(amr));
#endif
        }

        private void DrawPallets()
        {
#if UNITY_EDITOR
            foreach (var pallet in FindObjectsByType<PalletMarker>())
                Handles.Label(pallet.transform.position + ToVector3(palletLabelOffset), BuildPalletText(pallet));
#endif
        }

        private void DrawLoadingPoints()
        {
#if UNITY_EDITOR
            foreach (var loadingPoint in FindObjectsByType<PalletLoadingPoint>())
            {
                var position = PointPosition(loadingPoint.transform, loadingPoint.Node);
                Handles.Label(position + ToVector3(loadingPointLabelOffset), BuildLoadingPointText(loadingPoint));
            }
#endif
        }

        private void DrawWorkstations()
        {
#if UNITY_EDITOR
            foreach (var workstation in FindObjectsByType<WorkstationDeliveryPoint>())
            {
                var position = PointPosition(workstation.transform, workstation.Node);
                Handles.Label(position + ToVector3(workstationLabelOffset), BuildWorkstationText(workstation));
            }
#endif
        }

        private static Vector3 PointPosition(Transform fallback, Mapf.Authoring.MapfNode node)
        {
            return node != null ? node.transform.position : fallback.position;
        }

        private static Vector3 ToVector3(Vector2 offset)
        {
            return new Vector3(offset.x, offset.y, 0);
        }

        private static string BuildAmrText(TaskPlanningAmr amr)
        {
            var state = amr.IsBusy ? "Busy" : "Free";
            var kit = amr.AttachedPallet != null ? $"\nKit:{amr.AttachedPallet.KitId}" : string.Empty;
            return $"AMR:{amr.AmrId}\n{state}{kit}";
        }

        private static string BuildPalletText(PalletMarker pallet)
        {
            return $"Kit:{pallet.KitId}\n{pallet.LoadStateLabel}\n{pallet.Status}";
        }

        private static string BuildLoadingPointText(PalletLoadingPoint loadingPoint)
        {
            var reservation = loadingPoint.ReservedFor != null ? $"\nReserved:{loadingPoint.ReservedFor.KitId}" : string.Empty;
            var next = loadingPoint.NextQueuedPallet != null ? $"\nNext:{loadingPoint.NextQueuedPallet.KitId}" : string.Empty;
            return $"Load:{loadingPoint.LoadingPointId}\nKits:{loadingPoint.AcceptedPalletCount}\nQueue:{loadingPoint.QueueLength}{next}{reservation}";
        }

        private static string BuildWorkstationText(WorkstationDeliveryPoint workstation)
        {
            var reservation = workstation.ReservedFor != null ? $"\nReserved:{workstation.ReservedFor.KitId}" : string.Empty;
            var next = workstation.NextQueuedPallet != null ? $"\nNext:{workstation.NextQueuedPallet.KitId}" : string.Empty;
            return $"Unload:{workstation.WorkstationId}\nKits:{workstation.AcceptedPalletCount}\nQueue:{workstation.QueueLength}{next}{reservation}";
        }
    }
}
