using System.Collections.Generic;

namespace TaskPlanning
{
    public static class DispatchCandidateBuilder
    {
        public static IReadOnlyList<DispatchCandidate> BuildImmediateCandidates(DispatchProblem problem)
        {
            var candidates = new List<DispatchCandidate>();
            foreach (var task in problem.Tasks)
            {
                if (task == null)
                    continue;

                foreach (var amr in problem.Amrs)
                {
                    if (amr == null)
                        continue;

                    var startNode = problem.Distances.NearestNode(amr.transform.position);
                    var availability = new DispatchAvailability(
                        amr,
                        startNode,
                        problem.Now,
                        priorAssignmentEta: 0,
                        isImmediate: true);
                    var candidate = EvaluateCandidate(problem, task, availability);
                    if (candidate.IsValid)
                        candidates.Add(candidate);
                }
            }

            return candidates;
        }

        public static IReadOnlyList<DispatchCandidate> BuildFutureCandidates(DispatchProblem problem)
        {
            var candidates = new List<DispatchCandidate>();
            foreach (var task in problem.Tasks)
            {
                if (task == null)
                    continue;

                foreach (var future in problem.FutureAvailabilities)
                {
                    if (!future.IsValid)
                        continue;

                    var availability = new DispatchAvailability(
                        future.Amr,
                        future.FinishNode,
                        problem.Now + future.PriorAssignmentEta,
                        future.PriorAssignmentEta,
                        isImmediate: false);
                    var candidate = EvaluateCandidate(problem, task, availability);
                    if (candidate.IsValid)
                        candidates.Add(candidate);
                }
            }

            return candidates;
        }

        private static DispatchCandidate EvaluateCandidate(
            DispatchProblem problem,
            ITaskPlanningTask task,
            DispatchAvailability availability)
        {
            switch (task)
            {
                case DeliveryPlanningTask delivery:
                    return EvaluateDeliveryCandidate(problem, delivery, availability);
                case PalletRemovalPlanningTask removal:
                    return EvaluateRemovalCandidate(problem, removal, availability);
                default:
                    return default;
            }
        }

        private static DispatchCandidate EvaluateDeliveryCandidate(
            DispatchProblem problem,
            DeliveryPlanningTask task,
            DispatchAvailability availability)
        {
            var loadingPointResolution = problem.ResolveLoadingPoint(task.Pallet);
            if (!loadingPointResolution.IsResolved)
                return default;

            var loadingPoint = loadingPointResolution.LoadingPoint;
            var cost = availability.IsImmediate
                ? problem.CostEvaluator.Evaluate(availability.Amr, task, loadingPoint)
                : problem.CostEvaluator.EvaluateFrom(availability.StartNode, task, loadingPoint, availability.PriorAssignmentEta);
            if (!cost.IsFeasible)
                return default;

            return new DispatchCandidate(
                availability,
                task,
                task.Pallet,
                loadingPoint,
                task.Workstation,
                null,
                cost);
        }

        private static DispatchCandidate EvaluateRemovalCandidate(
            DispatchProblem problem,
            PalletRemovalPlanningTask task,
            DispatchAvailability availability)
        {
            var cost = availability.IsImmediate
                ? problem.CostEvaluator.Evaluate(availability.Amr, task)
                : problem.CostEvaluator.EvaluateFrom(availability.StartNode, task, availability.PriorAssignmentEta);
            if (!cost.IsFeasible)
                return default;

            return new DispatchCandidate(
                availability,
                task,
                task.Pallet,
                null,
                task.SourceWorkstation,
                task.Pallet.ParkingNode,
                cost);
        }
    }
}
