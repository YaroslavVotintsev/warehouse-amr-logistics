using System.Collections.Generic;

namespace TaskPlanning
{
    public sealed class DispatchProblem
    {
        public IReadOnlyList<ITaskPlanningTask> Tasks { get; }
        public IReadOnlyList<TaskPlanningAmr> Amrs { get; }
        public IReadOnlyList<AmrFutureAvailability> FutureAvailabilities { get; }
        public IReadOnlyList<PalletLoadingPoint> LoadingPoints { get; }
        public IReadOnlyList<DispatchCandidate> Candidates { get; }
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
            IReadOnlyList<AmrFutureAvailability> futureAvailabilities = null,
            IReadOnlyList<DispatchCandidate> candidates = null)
        {
            Tasks = tasks ?? System.Array.Empty<ITaskPlanningTask>();
            Amrs = amrs ?? System.Array.Empty<TaskPlanningAmr>();
            FutureAvailabilities = futureAvailabilities ?? System.Array.Empty<AmrFutureAvailability>();
            LoadingPoints = loadingPoints ?? System.Array.Empty<PalletLoadingPoint>();
            Distances = distances;
            CostEvaluator = costEvaluator;
            Now = now;
            Candidates = candidates ?? DispatchCandidateBuilder.BuildImmediateCandidates(this);
        }

        public PalletLoadingPointResolution ResolveLoadingPoint(PalletMarker pallet)
        {
            return PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, LoadingPoints);
        }

        public DispatchProblem WithCandidates(IReadOnlyList<DispatchCandidate> candidates)
        {
            return new DispatchProblem(
                Tasks,
                Amrs,
                LoadingPoints,
                Distances,
                CostEvaluator,
                Now,
                FutureAvailabilities,
                candidates);
        }
    }
}
