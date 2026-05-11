namespace TaskPlanning
{
    public interface ITaskPlanningTask
    {
        string TaskId { get; }
        PalletMarker Pallet { get; }
        TaskPlanningTaskType TaskType { get; }
        float EnqueuedTime { get; }
    }
}
