using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    public sealed class PalletLoadingPoint : MonoBehaviour
    {
        [SerializeField] private string loadingPointId;
        [SerializeField] private MapfNode node;
        [SerializeField] private PalletMarker[] acceptedPallets = System.Array.Empty<PalletMarker>();
        [SerializeField] private float loadDurationSeconds = 3f;

        private PalletMarker _reservedFor;

        public string LoadingPointId => string.IsNullOrWhiteSpace(loadingPointId) ? name : loadingPointId.Trim();
        public MapfNode Node => node;
        public float LoadDurationSeconds => Mathf.Max(0f, loadDurationSeconds);
        public int AcceptedPalletCount => acceptedPallets?.Length ?? 0;
        public bool IsReserved => _reservedFor != null;
        public PalletMarker ReservedFor => _reservedFor;

        public bool CanReserve(PalletMarker pallet)
        {
            return Accepts(pallet) && (_reservedFor == null || _reservedFor == pallet);
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
    }
}
