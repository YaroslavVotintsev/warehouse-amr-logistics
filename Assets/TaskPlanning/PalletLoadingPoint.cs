using Mapf.Authoring;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    public sealed class PalletLoadingPoint : MonoBehaviour
    {
        [SerializeField] private string loadingPointId;
        [SerializeField] private MapfNode node;
        [SerializeField] private PalletMarker[] acceptedPallets = System.Array.Empty<PalletMarker>();

        private PalletMarker _reservedFor;
        private readonly Queue<PalletMarker> _waitingPallets = new();

        public string LoadingPointId => string.IsNullOrWhiteSpace(loadingPointId) ? name : loadingPointId.Trim();
        public MapfNode Node => node;
        public int AcceptedPalletCount => acceptedPallets?.Length ?? 0;
        public bool IsReserved => _reservedFor != null;
        public PalletMarker ReservedFor => _reservedFor;
        public int QueueLength => _waitingPallets.Count;
        public PalletMarker NextQueuedPallet => _waitingPallets.Count > 0 ? _waitingPallets.Peek() : null;
        public IReadOnlyCollection<PalletMarker> QueuedPallets => _waitingPallets.ToArray();

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

        public void RemoveQueued(PalletMarker pallet)
        {
            if (pallet == null || _waitingPallets.Count == 0)
                return;

            var remaining = _waitingPallets.Where(p => p != pallet).ToArray();
            _waitingPallets.Clear();
            foreach (var queued in remaining)
                _waitingPallets.Enqueue(queued);
        }

        public bool Enqueue(PalletMarker pallet)
        {
            if (!Accepts(pallet))
                return false;

            if (_reservedFor == pallet || _waitingPallets.Contains(pallet))
                return true;

            _waitingPallets.Enqueue(pallet);
            return true;
        }

        public bool TryReserveNext(PalletMarker pallet)
        {
            if (!CanReserveNext(pallet))
                return false;

            _waitingPallets.Dequeue();
            _reservedFor = pallet;
            return true;
        }

        public bool CanReserveNext(PalletMarker pallet)
        {
            return Accepts(pallet) &&
                _reservedFor == null &&
                _waitingPallets.Count > 0 &&
                _waitingPallets.Peek() == pallet;
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
