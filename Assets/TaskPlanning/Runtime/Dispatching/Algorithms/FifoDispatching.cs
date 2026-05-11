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
            var best = default(DispatchAssignment);
            foreach (var amr in problem.Amrs)
            {
                if (amr == null || amr.IsBusy || usedAmrs.Contains(amr))
                    continue;

                var candidate = EvaluateCandidate(problem, task, amr);
                if (!candidate.IsValid || !candidate.Cost.IsFeasible)
                    continue;

                if (!best.IsValid || candidate.Score < best.Score)
                    best = candidate;
            }

            return best;
        }

        private static DispatchAssignment EvaluateCandidate(
            DispatchProblem problem,
            ITaskPlanningTask task,
            TaskPlanningAmr amr)
        {
            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    return EvaluateDeliveryCandidate(problem, delivery, amr);
                case PalletRemovalPlanningTask removal:
                    return EvaluateRemovalCandidate(problem, removal, amr);
                default:
                    return default;
            }
        }

        private static DispatchAssignment EvaluateDeliveryCandidate(
            DispatchProblem problem,
            DeliveryPlanningTask task,
            TaskPlanningAmr amr)
        {
            var loadingPointResolution = problem.ResolveLoadingPoint(task.Pallet);
            if (!loadingPointResolution.IsResolved)
                return default;

            var loadingPoint = loadingPointResolution.LoadingPoint;
            var cost = problem.CostEvaluator.Evaluate(amr, task, loadingPoint);
            if (!cost.IsFeasible)
                return default;

            return new DispatchAssignment(
                amr,
                task,
                task.Pallet,
                loadingPoint,
                task.Workstation,
                null,
                cost);
        }

        private static DispatchAssignment EvaluateRemovalCandidate(
            DispatchProblem problem,
            PalletRemovalPlanningTask task,
            TaskPlanningAmr amr)
        {
            var cost = problem.CostEvaluator.Evaluate(amr, task);
            if (!cost.IsFeasible)
                return default;

            return new DispatchAssignment(
                amr,
                task,
                task.Pallet,
                null,
                task.SourceWorkstation,
                task.Pallet.ParkingNode,
                cost);
        }
    }
}
