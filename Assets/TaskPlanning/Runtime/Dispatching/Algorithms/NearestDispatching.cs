using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public class NearestDispatching : ITaskDispatchAlgorithm
    {
        public virtual DispatchPlan Solve(DispatchProblem problem)
        {
            return SelectGreedy(problem.Candidates);
        }

        protected static DispatchPlan SelectGreedy(IEnumerable<DispatchCandidate> candidates)
        {
            var selected = new List<DispatchAssignment>();
            var usedAmrs = new HashSet<TaskPlanningAmr>();
            var usedTasks = new HashSet<ITaskPlanningTask>();
            var usedPallets = new HashSet<PalletMarker>();

            foreach (var candidate in candidates.OrderBy(c => c.Score))
            {
                if (usedAmrs.Contains(candidate.Amr) ||
                    usedTasks.Contains(candidate.Task) ||
                    usedPallets.Contains(candidate.Pallet))
                    continue;

                usedAmrs.Add(candidate.Amr);
                usedTasks.Add(candidate.Task);
                usedPallets.Add(candidate.Pallet);
                selected.Add(candidate.ToAssignment());
            }

            return new DispatchPlan(selected);
        }
    }

    public sealed class GreedyNearestFeasibleDispatchAlgorithm : NearestDispatching
    {
    }
}
