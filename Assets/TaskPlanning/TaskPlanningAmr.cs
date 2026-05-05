using Mapf.Authoring;
using UnityEngine;

namespace TaskPlanning
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(MapfAgent))]
    public sealed class TaskPlanningAmr : MonoBehaviour
    {
        [SerializeField] private string amrId;
        [SerializeField] private Transform palletMount;

        public string AmrId => string.IsNullOrWhiteSpace(amrId) ? name : amrId.Trim();
        public Transform PalletMount => palletMount != null ? palletMount : transform;
        public MapfAgent MapfAgent
        {
            get
            {
                _mapfAgent ??= GetComponent<MapfAgent>();
                return _mapfAgent;
            }
        }
        public bool IsBusy { get; private set; }
        public PalletMarker AttachedPallet { get; private set; }

        private MapfAgent _mapfAgent;

        private void Awake()
        {
            _mapfAgent = GetComponent<MapfAgent>();
        }

        public bool TryReserve()
        {
            if (IsBusy)
                return false;

            IsBusy = true;
            return true;
        }

        public void Attach(PalletMarker pallet)
        {
            AttachedPallet = pallet;
        }

        public void Release()
        {
            IsBusy = false;
            AttachedPallet = null;
        }
    }
}
