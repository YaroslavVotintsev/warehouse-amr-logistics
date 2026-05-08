using System.Collections.Generic;

namespace TaskPlanning
{
    public sealed class GreedyNearestFeasibleDispatchAlgorithm : ITaskDispatchAlgorithm
    {
        public DispatchAssignment SelectAssignment(
            DeliveryTaskRequest request,
            IReadOnlyList<TaskPlanningAmr> amrs,
            IReadOnlyList<PalletLoadingPoint> loadingPoints,
            RoadmapDistanceService distances)
        {
            var best = default(DispatchAssignment);
            var bestScore = double.PositiveInfinity;
            var pallet = request.pallet;
            if (pallet == null || !pallet.IsAvailable || pallet.CurrentNode == null)
                return best;

            foreach (var amr in amrs)
            {
                if (amr == null || amr.IsBusy)
                    continue;

                var amrNode = distances.NearestNode(amr.transform.position);
                if (amrNode == null)
                    continue;

                foreach (var loadingPoint in loadingPoints)
                {
                    if (loadingPoint == null || loadingPoint.Node == null || !loadingPoint.Accepts(pallet))
                        continue;

                    if (request.workstation == null || request.workstation.Node == null || !request.workstation.Accepts(pallet))
                        continue;

                    var toPallet = distances.Distance(amrNode, pallet.CurrentNode);
                    var toLoad = distances.Distance(pallet.CurrentNode, loadingPoint.Node);
                    var toWorkstation = distances.Distance(loadingPoint.Node, request.workstation.Node);
                    if (double.IsPositiveInfinity(toPallet) || double.IsPositiveInfinity(toLoad) || double.IsPositiveInfinity(toWorkstation))
                        continue;

                    var score = toPallet + toLoad + toWorkstation;
                    if (score >= bestScore)
                        continue;

                    bestScore = score;
                    best = new DispatchAssignment(amr, pallet, loadingPoint, score);
                }
            }

            return best;
        }
    }
}
