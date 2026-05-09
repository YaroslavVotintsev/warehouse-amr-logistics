namespace TaskPlanning
{
    public sealed class DeliveryPlanningTask : ITaskPlanningTask
    {
        public string TaskId { get; }
        public PalletMarker Pallet { get; }
        public WorkstationDeliveryPoint Workstation { get; }
        public float EnqueuedTime { get; }
        public TaskPlanningTaskType TaskType => TaskPlanningTaskType.Delivery;

        public DeliveryPlanningTask(string taskId, PalletMarker pallet, WorkstationDeliveryPoint workstation, float enqueuedTime)
        {
            TaskId = taskId;
            Pallet = pallet;
            Workstation = workstation;
            EnqueuedTime = enqueuedTime;
        }
    }
}
