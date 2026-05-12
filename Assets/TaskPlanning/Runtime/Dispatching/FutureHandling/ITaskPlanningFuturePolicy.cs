namespace TaskPlanning
{
    public interface ITaskPlanningFuturePolicy
    {
        DispatchPlan Solve(DispatchProblem problem, ITaskDispatchAlgorithm baseDispatcher);
    }
}
