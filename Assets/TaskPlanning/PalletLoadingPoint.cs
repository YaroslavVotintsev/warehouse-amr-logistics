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

        public string LoadingPointId => string.IsNullOrWhiteSpace(loadingPointId) ? name : loadingPointId.Trim();
        public MapfNode Node => node;
        public float LoadDurationSeconds => Mathf.Max(0f, loadDurationSeconds);

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
