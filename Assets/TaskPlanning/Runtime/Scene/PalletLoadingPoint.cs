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

        public void Configure(string id, MapfNode mapfNode, IEnumerable<PalletMarker> accepted)
        {
            loadingPointId = id;
            node = mapfNode;
            acceptedPallets = accepted?.Where(pallet => pallet != null).ToArray() ?? System.Array.Empty<PalletMarker>();
            _reservedFor = null;
            _waitingPallets.Clear();
            if (node != null)
                transform.position = node.transform.position;
        }

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

        public static PalletLoadingPointResolution ResolveAcceptedLoadingPoint(
            PalletMarker pallet,
            IEnumerable<PalletLoadingPoint> loadingPoints)
        {
            if (pallet == null)
                return PalletLoadingPointResolution.Failed(
                    PalletLoadingPointResolutionStatus.MissingPallet,
                    null,
                    "Cannot resolve loading point because the pallet/kit is missing.");

            var matches = new List<PalletLoadingPoint>();
            var seen = new HashSet<PalletLoadingPoint>();
            if (loadingPoints != null)
            {
                foreach (var loadingPoint in loadingPoints)
                {
                    if (loadingPoint == null || !seen.Add(loadingPoint) || !loadingPoint.Accepts(pallet))
                        continue;

                    matches.Add(loadingPoint);
                }
            }

            if (matches.Count == 0)
                return PalletLoadingPointResolution.Failed(
                    PalletLoadingPointResolutionStatus.NoAcceptingLoadingPoint,
                    null,
                    $"No loading point accepts pallet/kit '{pallet.KitId}'.");

            if (matches.Count > 1)
            {
                var loadingPointNames = string.Join(", ", matches.Select(point => point.LoadingPointId));
                return PalletLoadingPointResolution.Failed(
                    PalletLoadingPointResolutionStatus.MultipleAcceptingLoadingPoints,
                    null,
                    $"Pallet/kit '{pallet.KitId}' is accepted by multiple loading points: {loadingPointNames}. Configure it on exactly one loading point.");
            }

            var resolved = matches[0];
            if (resolved.Node == null)
                return PalletLoadingPointResolution.Failed(
                    PalletLoadingPointResolutionStatus.MissingNode,
                    resolved,
                    $"Loading point '{resolved.LoadingPointId}' accepts pallet/kit '{pallet.KitId}' but has no node.");

            return PalletLoadingPointResolution.Resolved(resolved);
        }
    }

    public enum PalletLoadingPointResolutionStatus
    {
        Resolved,
        MissingPallet,
        NoAcceptingLoadingPoint,
        MultipleAcceptingLoadingPoints,
        MissingNode
    }

    public readonly struct PalletLoadingPointResolution
    {
        public readonly PalletLoadingPointResolutionStatus Status;
        public readonly PalletLoadingPoint LoadingPoint;
        public readonly string Message;

        private PalletLoadingPointResolution(
            PalletLoadingPointResolutionStatus status,
            PalletLoadingPoint loadingPoint,
            string message)
        {
            Status = status;
            LoadingPoint = loadingPoint;
            Message = message;
        }

        public bool IsResolved => Status == PalletLoadingPointResolutionStatus.Resolved;

        public static PalletLoadingPointResolution Resolved(PalletLoadingPoint loadingPoint)
        {
            return new PalletLoadingPointResolution(
                PalletLoadingPointResolutionStatus.Resolved,
                loadingPoint,
                string.Empty);
        }

        public static PalletLoadingPointResolution Failed(
            PalletLoadingPointResolutionStatus status,
            PalletLoadingPoint loadingPoint,
            string message)
        {
            return new PalletLoadingPointResolution(status, loadingPoint, message);
        }
    }
}
