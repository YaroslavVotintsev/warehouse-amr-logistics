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

        private PalletMarker _reservedFor;

        public string WorkstationId => string.IsNullOrWhiteSpace(workstationId) ? name : workstationId.Trim();
        public MapfNode Node => node;
        public float UnloadDurationSeconds => Mathf.Max(0f, unloadDurationSeconds);
        public int AcceptedPalletCount => acceptedPallets?.Length ?? 0;
        public bool IsReserved => _reservedFor != null;
        public PalletMarker ReservedFor => _reservedFor;

        public bool CanReserve(PalletMarker pallet)
        {
            return Accepts(pallet) &&
                (_reservedFor == null || _reservedFor == pallet) &&
                !HasBlockingPallet(pallet);
        }

        public bool TryReserve(PalletMarker pallet)
        {
            if (!CanReserve(pallet))
                return false;

            _reservedFor = pallet;
            return true;
        }

        public void ReleaseReservation(PalletMarker pallet)
        {
            if (_reservedFor == pallet)
                _reservedFor = null;
        }

        public bool Accepts(PalletMarker pallet)
        {
            if (pallet == null)
                return false;

            foreach (var accepted in acceptedPallets)
                if (accepted == pallet)
                    return true;

            return false;
        }

        private bool HasBlockingPallet(PalletMarker incomingPallet)
        {
            if (node == null)
                return true;

            foreach (var pallet in FindObjectsByType<PalletMarker>(FindObjectsInactive.Exclude))
            {
                if (pallet == null || pallet == incomingPallet)
                    continue;

                if (pallet.CurrentNode == node)
                    return true;
            }

            return false;
        }
    }
}
