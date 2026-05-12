using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public sealed class FifoDispatching : ITaskDispatchAlgorithm
    {
        public DispatchPlan Solve(DispatchProblem problem)
        {
            var selected = new List<DispatchAssignment>();
            var usedAmrs = new HashSet<TaskPlanningAmr>();
            var usedPallets = new HashSet<PalletMarker>();

            foreach (var task in problem.Tasks.Where(task => task != null).OrderBy(task => task.EnqueuedTime))
            {
                if (task.Pallet == null || usedPallets.Contains(task.Pallet))
                    continue;

                var best = BestAssignmentForTask(problem, task, usedAmrs);
                if (!best.IsValid)
                    continue;

                selected.Add(best);
                usedAmrs.Add(best.Amr);
                usedPallets.Add(best.Pallet);
            }

            return new DispatchPlan(selected);
        }

        private static DispatchAssignment BestAssignmentForTask(
            DispatchProblem problem,
            ITaskPlanningTask task,
            HashSet<TaskPlanningAmr> usedAmrs)
        {
            var best = default(DispatchCandidate);
            foreach (var candidate in problem.Candidates)
            {
                if (!candidate.IsValid ||
                    candidate.Task != task ||
                    candidate.Amr == null ||
                    candidate.Amr.IsBusy ||
                    usedAmrs.Contains(candidate.Amr))
                    continue;

                if (!best.IsValid || candidate.Score < best.Score)
                    best = candidate;
            }

            return best.IsValid ? best.ToAssignment() : default;
        }
    }
}
