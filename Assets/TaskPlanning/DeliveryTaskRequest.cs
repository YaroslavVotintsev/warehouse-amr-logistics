using System;

namespace TaskPlanning
{
    [Serializable]
    public sealed class DeliveryTaskRequest
    {
        public string taskId;
        public PalletMarker pallet;
        public WorkstationDeliveryPoint workstation;

        public DeliveryTaskRequest()
        {
        }

        public DeliveryTaskRequest(string taskId, PalletMarker pallet, WorkstationDeliveryPoint workstation)
        {
            this.taskId = taskId;
            this.pallet = pallet;
            this.workstation = workstation;
        }
    }
}
