using UnityEngine;

namespace TaskPlanning
{
    [System.Serializable]
    public sealed class TaskPlanningCostWeights : ISerializationCallbackReceiver
    {
        private const int CurrentSerializedVersion = 1;

        [SerializeField, HideInInspector] private int serializedVersion = CurrentSerializedVersion;
        [SerializeField, HideInInspector] private float priorAssignmentEta = 1f;
        [SerializeField, HideInInspector] private float amrToPalletEta = 1f;
        [SerializeField, HideInInspector] private float attachTime = 1f;
        [SerializeField, HideInInspector] private float loadingQueueEta = 1f;
        [SerializeField, HideInInspector] private float palletToLoadingEta = 1f;
        [SerializeField, HideInInspector] private float loadTime = 1f;
        [SerializeField, HideInInspector] private float loadingToWorkstationEta = 1f;
        [SerializeField, HideInInspector] private float detachTime = 1f;
        [SerializeField, HideInInspector] private float palletToParkingEta = 1f;
        [SerializeField, HideInInspector] private float removalBlocksPendingDeliveryMultiplier = 0.5f;
        [SerializeField, HideInInspector] private float blockedDeliveryBias = 100f;
        [SerializeField, HideInInspector] private float agingWeight = 1f;
        [SerializeField, HideInInspector] private float maxAgingBonus = 300f;

        public DeliveryTaskCostWeights delivery = new();
        public RemovalTaskCostWeights removal = new();

        public void OnBeforeSerialize()
        {
            serializedVersion = CurrentSerializedVersion;
        }

        public void OnAfterDeserialize()
        {
            delivery ??= new DeliveryTaskCostWeights();
            removal ??= new RemovalTaskCostWeights();

            if (serializedVersion >= CurrentSerializedVersion)
                return;

            delivery.amrToPalletEta = amrToPalletEta;
            delivery.priorAssignmentEta = priorAssignmentEta;
            delivery.attachTime = attachTime;
            delivery.loadingQueueEta = loadingQueueEta;
            delivery.palletToLoadingEta = palletToLoadingEta;
            delivery.loadTime = loadTime;
            delivery.loadingToWorkstationEta = loadingToWorkstationEta;
            delivery.detachTime = detachTime;
            delivery.blockedDeliveryBias = blockedDeliveryBias;
            delivery.agingWeight = agingWeight;
            delivery.maxAgingBonus = maxAgingBonus;

            removal.amrToPalletEta = amrToPalletEta;
            removal.priorAssignmentEta = priorAssignmentEta;
            removal.attachTime = attachTime;
            removal.palletToParkingEta = palletToParkingEta;
            removal.detachTime = detachTime;
            removal.blocksPendingDeliveryMultiplier = removalBlocksPendingDeliveryMultiplier;
            removal.agingWeight = agingWeight;
            removal.maxAgingBonus = maxAgingBonus;
        }
    }

    [System.Serializable]
    public sealed class DeliveryTaskCostWeights
    {
        [Min(0f)] public float priorAssignmentEta = 1f;
        [Min(0f)] public float amrToPalletEta = 1f;
        [Min(0f)] public float attachTime = 1f;
        [Min(0f)] public float loadingQueueEta = 1f;
        [Min(0f)] public float palletToLoadingEta = 1f;
        [Min(0f)] public float loadTime = 1f;
        [Min(0f)] public float loadingToWorkstationEta = 1f;
        [Min(0f)] public float detachTime = 1f;
        [Min(0f)] public float blockedDeliveryBias = 100f;
        [Min(0f)] public float agingWeight = 1f;
        [Min(0f)] public float maxAgingBonus = 300f;
    }

    [System.Serializable]
    public sealed class RemovalTaskCostWeights
    {
        [Min(0f)] public float priorAssignmentEta = 1f;
        [Min(0f)] public float amrToPalletEta = 1f;
        [Min(0f)] public float attachTime = 1f;
        [Min(0f)] public float palletToParkingEta = 1f;
        [Min(0f)] public float detachTime = 1f;
        [Min(0f)] public float blocksPendingDeliveryMultiplier = 0.5f;
        [Min(0f)] public float agingWeight = 1f;
        [Min(0f)] public float maxAgingBonus = 300f;
    }
}
