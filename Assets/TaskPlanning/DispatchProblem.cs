using System.Collections.Generic;

namespace TaskPlanning
{
    public sealed class DispatchProblem
    {
        public IReadOnlyList<ITaskPlanningTask> Tasks { get; }
        public IReadOnlyList<TaskPlanningAmr> Amrs { get; }
        public IReadOnlyList<AmrFutureAvailability> FutureAvailabilities { get; }
        public IReadOnlyList<PalletLoadingPoint> LoadingPoints { get; }
        public RoadmapDistanceService Distances { get; }
        public TaskPlanningCostEvaluator CostEvaluator { get; }
        public float Now { get; }

        public DispatchProblem(
            IReadOnlyList<ITaskPlanningTask> tasks,
            IReadOnlyList<TaskPlanningAmr> amrs,
            IReadOnlyList<PalletLoadingPoint> loadingPoints,
            RoadmapDistanceService distances,
            TaskPlanningCostEvaluator costEvaluator,
            float now,
            IReadOnlyList<AmrFutureAvailability> futureAvailabilities = null)
        {
            Tasks = tasks;
            Amrs = amrs;
            FutureAvailabilities = futureAvailabilities ?? System.Array.Empty<AmrFutureAvailability>();
            LoadingPoints = loadingPoints;
            Distances = distances;
            CostEvaluator = costEvaluator;
            Now = now;
        }
    }
}
