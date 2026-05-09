using UnityEngine;

namespace TaskPlanning
{
    [System.Serializable]
    public sealed class TaskPlanningCostWeights
    {
        [Min(0f)] public float priorAssignmentEta = 1f;
        [Min(0f)] public float amrToPalletEta = 1f;
        [Min(0f)] public float attachTime = 1f;
        [Min(0f)] public float loadingQueueEta = 1f;
        [Min(0f)] public float palletToLoadingEta = 1f;
        [Min(0f)] public float loadTime = 1f;
        [Min(0f)] public float loadingToWorkstationEta = 1f;
        [Min(0f)] public float detachTime = 1f;
        [Min(0f)] public float palletToParkingEta = 1f;
        [Min(0f)] public float removalBlocksPendingDeliveryMultiplier = 0.5f;
        [Min(0f)] public float blockedDeliveryBias = 100f;
        [Min(0f)] public float agingWeight = 1f;
        [Min(0f)] public float maxAgingBonus = 300f;
    }
}
