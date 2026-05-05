using System.Collections.Generic;

namespace TaskPlanning
{
    public interface ITaskDispatchAlgorithm
    {
        DispatchAssignment SelectAssignment(
            DeliveryTaskRequest request,
            IReadOnlyList<TaskPlanningAmr> amrs,
            IReadOnlyList<PalletLoadingPoint> loadingPoints,
            RoadmapDistanceService distances);
    }
}
