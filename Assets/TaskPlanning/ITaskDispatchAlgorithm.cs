using System.Collections.Generic;

namespace TaskPlanning
{
    public interface ITaskDispatchAlgorithm
    {
        DispatchPlan Solve(DispatchProblem problem);
    }
}
