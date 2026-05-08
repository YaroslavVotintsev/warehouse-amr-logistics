using Mapf.Authoring;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    public sealed class WorkstationDeliveryPoint : MonoBehaviour
    {
        [SerializeField] private string workstationId;
        [SerializeField] private MapfNode node;
        [SerializeField] private PalletMarker[] acceptedPallets = System.Array.Empty<PalletMarker>();
        [SerializeField] private float palletOccupancyDistance = 0.12f;

        private PalletMarker _reservedFor;
        private readonly Queue<PalletMarker> _waitingPallets = new();

        public string WorkstationId => string.IsNullOrWhiteSpace(workstationId) ? name : workstationId.Trim();
        public MapfNode Node => node;
        public int AcceptedPalletCount => acceptedPallets?.Length ?? 0;
        public bool IsReserved => _reservedFor != null;
        public PalletMarker ReservedFor => _reservedFor;
        public int QueueLength => _waitingPallets.Count;
        public PalletMarker NextQueuedPallet => _waitingPallets.Count > 0 ? _waitingPallets.Peek() : null;

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
                !HasBlockingPallet(pallet) &&
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

        private bool HasBlockingPallet(PalletMarker incomingPallet)
        {
            if (node == null)
                return true;

            foreach (var pallet in FindObjectsByType<PalletMarker>(FindObjectsInactive.Exclude))
            {
                if (pallet == null || pallet == incomingPallet)
                    continue;

                if (!pallet.IsAvailable)
                    continue;

                var distance = Vector2.Distance(pallet.transform.position, node.transform.position);
                if (distance <= palletOccupancyDistance)
                    return true;
            }

            return false;
        }
    }
}
