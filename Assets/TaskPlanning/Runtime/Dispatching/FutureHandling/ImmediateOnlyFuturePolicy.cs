namespace TaskPlanning
{
    public sealed class ImmediateOnlyFuturePolicy : ITaskPlanningFuturePolicy
    {
        public DispatchPlan Solve(DispatchProblem problem, ITaskDispatchAlgorithm baseDispatcher)
        {
            var immediateCandidates = DispatchCandidateBuilder.BuildImmediateCandidates(problem);
            var immediateProblem = problem.WithCandidates(immediateCandidates);
            return baseDispatcher.Solve(immediateProblem);
        }
    }
}
