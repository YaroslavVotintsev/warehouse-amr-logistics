using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    public sealed class WorkstationDeliveryPoint : MonoBehaviour
    {
        [SerializeField] private string workstationId;
        [SerializeField] private MapfNode node;
        [SerializeField] private PalletMarker[] acceptedPallets = System.Array.Empty<PalletMarker>();
        [SerializeField] private float unloadDurationSeconds = 3f;

        public string WorkstationId => string.IsNullOrWhiteSpace(workstationId) ? name : workstationId.Trim();
        public MapfNode Node => node;
        public float UnloadDurationSeconds => Mathf.Max(0f, unloadDurationSeconds);
        public int AcceptedPalletCount => acceptedPallets?.Length ?? 0;

        public bool Accepts(PalletMarker pallet)
        {
            if (pallet == null)
                return false;

            foreach (var accepted in acceptedPallets)
                if (accepted == pallet)
                    return true;

            return false;
        }
    }
}
