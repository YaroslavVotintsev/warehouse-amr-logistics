using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    public enum PalletStatus
    {
        Available,
        Reserved,
        Attached,
        Loading,
        Loaded,
        Unloading
    }

    [DisallowMultipleComponent]
    public sealed class PalletMarker : MonoBehaviour
    {
        [SerializeField] private string palletId;
        [SerializeField] private MapfNode currentNode;

        private Transform _originalParent;

        public string PalletId => string.IsNullOrWhiteSpace(palletId) ? name : palletId.Trim();
        public string KitId => PalletId;
        public MapfNode CurrentNode => currentNode;
        public PalletStatus Status { get; private set; } = PalletStatus.Available;
        public bool IsAvailable => Status == PalletStatus.Available;

        private void Awake()
        {
            _originalParent = transform.parent;
        }

        public bool TryReserve()
        {
            if (!IsAvailable)
                return false;

            Status = PalletStatus.Reserved;
            return true;
        }

        public void ReleaseReservation()
        {
            if (Status == PalletStatus.Reserved)
                Status = PalletStatus.Available;
        }

        public void AttachTo(TaskPlanningAmr amr)
        {
            Status = PalletStatus.Attached;
            transform.SetParent(amr.PalletMount, true);
            transform.localPosition = Vector3.zero;
            amr.Attach(this);
        }

        public void MarkLoading()
        {
            Status = PalletStatus.Loading;
        }

        public void MarkLoaded()
        {
            Status = PalletStatus.Loaded;
        }

        public void MarkUnloading()
        {
            Status = PalletStatus.Unloading;
        }

        public void DetachAt(MapfNode node)
        {
            currentNode = node;
            Status = PalletStatus.Available;
            transform.SetParent(_originalParent, true);
            if (node != null)
                transform.position = new Vector3(node.transform.position.x, node.transform.position.y, transform.position.z);
        }
    }
}
