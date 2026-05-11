using System.Collections.Generic;
using System.Linq;

namespace TaskPlanning
{
    public class NearestDispatching : ITaskDispatchAlgorithm
    {
        public virtual DispatchPlan Solve(DispatchProblem problem)
        {
            var candidates = new List<DispatchAssignment>();
            foreach (var task in problem.Tasks)
                candidates.AddRange(BuildImmediateCandidates(problem, task));

            return SelectGreedy(candidates);
        }

        protected static DispatchPlan SelectGreedy(IEnumerable<DispatchAssignment> candidates)
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
                selected.Add(candidate);
            }

            return new DispatchPlan(selected);
        }

        protected static List<DispatchAssignment> BuildImmediateCandidates(DispatchProblem problem, ITaskPlanningTask task)
        {
            var candidates = new List<DispatchAssignment>();
            foreach (var amr in problem.Amrs)
            {
                if (amr == null)
                    continue;

                AddCandidates(problem, task, amr, candidates);
            }

            return candidates;
        }

        protected static void AddCandidates(
            DispatchProblem problem,
            ITaskPlanningTask task,
            TaskPlanningAmr amr,
            ICollection<DispatchAssignment> candidates)
        {
            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    AddDeliveryCandidates(problem, delivery, amr, candidates);
                    break;
                case PalletRemovalPlanningTask removal:
                    AddRemovalCandidate(problem, removal, amr, candidates);
                    break;
            }
        }

        protected static void AddDeliveryCandidates(
            DispatchProblem problem,
            DeliveryPlanningTask task,
            TaskPlanningAmr amr,
            ICollection<DispatchAssignment> candidates)
        {
            var loadingPointResolution = problem.ResolveLoadingPoint(task.Pallet);
            if (!loadingPointResolution.IsResolved)
                return;

            var loadingPoint = loadingPointResolution.LoadingPoint;
            var cost = problem.CostEvaluator.Evaluate(amr, task, loadingPoint);
            if (!cost.IsFeasible)
                return;

            candidates.Add(new DispatchAssignment(
                amr,
                task,
                task.Pallet,
                loadingPoint,
                task.Workstation,
                null,
                cost));
        }

        protected static void AddRemovalCandidate(
            DispatchProblem problem,
            PalletRemovalPlanningTask task,
            TaskPlanningAmr amr,
            ICollection<DispatchAssignment> candidates)
        {
            var cost = problem.CostEvaluator.Evaluate(amr, task);
            if (!cost.IsFeasible)
                return;

            candidates.Add(new DispatchAssignment(
                amr,
                task,
                task.Pallet,
                null,
                task.SourceWorkstation,
                task.Pallet.ParkingNode,
                cost));
        }
    }

    public sealed class GreedyNearestFeasibleDispatchAlgorithm : NearestDispatching
    {
    }
}
