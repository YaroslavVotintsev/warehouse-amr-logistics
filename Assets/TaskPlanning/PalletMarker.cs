using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    public enum PalletStatus
    {
        Available,
        Reserved,
        Attaching,
        Attached,
        Loading,
        Loaded,
        Unloading,
        Detaching
    }

    [DisallowMultipleComponent]
    public sealed class PalletMarker : MonoBehaviour
    {
        [SerializeField] private string palletId;
        [SerializeField] private MapfNode currentNode;
        [SerializeField] private float attachDurationSeconds = 1f;
        [SerializeField] private float detachDurationSeconds = 1f;
        [SerializeField] private float loadDurationSeconds = 3f;
        [SerializeField] private float unloadDurationSeconds = 3f;

        private Transform _originalParent;
        private bool _isLoaded;

        public string PalletId => string.IsNullOrWhiteSpace(palletId) ? name : palletId.Trim();
        public string KitId => PalletId;
        public MapfNode CurrentNode => currentNode;
        public float AttachDurationSeconds => Mathf.Max(0f, attachDurationSeconds);
        public float DetachDurationSeconds => Mathf.Max(0f, detachDurationSeconds);
        public float LoadDurationSeconds => Mathf.Max(0f, loadDurationSeconds);
        public float UnloadDurationSeconds => Mathf.Max(0f, unloadDurationSeconds);
        public PalletStatus Status { get; private set; } = PalletStatus.Available;
        public bool IsAvailable => Status == PalletStatus.Available;
        public bool IsLoaded => _isLoaded;
        public string LoadStateLabel => _isLoaded ? "Loaded" : "Unloaded";

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

        public void MarkAttaching()
        {
            Status = PalletStatus.Attaching;
        }

        public void AttachTo(TaskPlanningAmr amr)
        {
            Status = PalletStatus.Attached;
            currentNode = null;
            transform.SetParent(amr.PalletMount, true);
            transform.localPosition = Vector3.zero;
            amr.Attach(this);
        }

        public void MarkLoading()
        {
            _isLoaded = false;
            Status = PalletStatus.Loading;
        }

        public void MarkLoaded()
        {
            _isLoaded = true;
            Status = PalletStatus.Loaded;
        }

        public void MarkUnloading()
        {
            Status = PalletStatus.Unloading;
        }

        public void MarkDetaching()
        {
            Status = PalletStatus.Detaching;
        }

        public void DetachAt(MapfNode node)
        {
            DetachAt(node, PalletStatus.Available);
        }

        public void DetachAt(MapfNode node, PalletStatus statusAfterDetach)
        {
            currentNode = node;
            Status = statusAfterDetach;
            transform.SetParent(_originalParent, true);
            if (node != null)
                transform.position = new Vector3(node.transform.position.x, node.transform.position.y, transform.position.z);
        }

        public void MarkUnloadedAvailable()
        {
            _isLoaded = false;
            Status = PalletStatus.Available;
        }
    }
}
