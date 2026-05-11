namespace TaskPlanning
{
    public sealed class PalletRemovalPlanningTask : ITaskPlanningTask
    {
        public string TaskId { get; }
        public PalletMarker Pallet { get; }
        public WorkstationDeliveryPoint SourceWorkstation { get; }
        public float EnqueuedTime { get; }
        public TaskPlanningTaskType TaskType => TaskPlanningTaskType.PalletRemoval;

        public PalletRemovalPlanningTask(string taskId, PalletMarker pallet, WorkstationDeliveryPoint sourceWorkstation, float enqueuedTime)
        {
            TaskId = taskId;
            Pallet = pallet;
            SourceWorkstation = sourceWorkstation;
            EnqueuedTime = enqueuedTime;
        }
    }
}
