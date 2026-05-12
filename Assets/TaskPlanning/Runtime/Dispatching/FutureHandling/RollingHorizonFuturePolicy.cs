namespace TaskPlanning
{
    public sealed class RollingHorizonFuturePolicy : ITaskPlanningFuturePolicy
    {
        private readonly ImmediateOnlyFuturePolicy _fallback = new();

        public DispatchPlan Solve(DispatchProblem problem, ITaskDispatchAlgorithm baseDispatcher)
        {
            return _fallback.Solve(problem, baseDispatcher);
        }
    }
}
