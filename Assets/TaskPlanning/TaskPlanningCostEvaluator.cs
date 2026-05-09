using Mapf.Authoring;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class TaskPlanningCostEvaluator
    {
        private readonly RoadmapDistanceService _distances;
        private readonly TaskPlanningCostWeights _weights;
        private readonly float _amrSpeed;
        private readonly IReadOnlyList<ITaskPlanningTask> _pendingTasks;
        private readonly float _now;

        public TaskPlanningCostEvaluator(
            RoadmapDistanceService distances,
            TaskPlanningCostWeights weights,
            float amrSpeed,
            IReadOnlyList<ITaskPlanningTask> pendingTasks,
            float now)
        {
            _distances = distances;
            _weights = weights;
            _amrSpeed = Mathf.Max(0.0001f, amrSpeed);
            _pendingTasks = pendingTasks ?? System.Array.Empty<ITaskPlanningTask>();
            _now = now;
        }

        public CostEvaluation Evaluate(TaskPlanningAmr amr, DeliveryPlanningTask task, PalletLoadingPoint loadingPoint)
        {
            if (amr == null || task?.Pallet == null || task.Workstation == null || loadingPoint == null)
                return CostEvaluation.Infeasible;

            var pallet = task.Pallet;
            if (!pallet.IsAvailable || pallet.CurrentNode == null || loadingPoint.Node == null || task.Workstation.Node == null)
                return CostEvaluation.Infeasible;

            if (!loadingPoint.Accepts(pallet) || !task.Workstation.Accepts(pallet))
                return CostEvaluation.Infeasible;

            var amrNode = _distances.NearestNode(amr.transform.position);
            if (amrNode == null)
                return CostEvaluation.Infeasible;

            var amrToPallet = Eta(amrNode, pallet.CurrentNode);
            var palletToLoading = Eta(pallet.CurrentNode, loadingPoint.Node);
            var loadingToWorkstation = Eta(loadingPoint.Node, task.Workstation.Node);
            if (!Finite(amrToPallet) || !Finite(palletToLoading) || !Finite(loadingToWorkstation))
                return CostEvaluation.Infeasible;

            var loadingQueueEta = EstimateLoadingQueueEta(loadingPoint);
            var blockedDeliveryBias = task.Workstation.HasBlockingPalletFor(pallet)
                ? _weights.blockedDeliveryBias
                : 0.0;
            var agingBonus = AgingBonus(task);
            var total =
                _weights.amrToPalletEta * amrToPallet +
                _weights.attachTime * pallet.AttachDurationSeconds +
                _weights.loadingQueueEta * loadingQueueEta +
                _weights.palletToLoadingEta * palletToLoading +
                _weights.loadTime * pallet.LoadDurationSeconds +
                _weights.loadingToWorkstationEta * loadingToWorkstation +
                _weights.detachTime * pallet.DetachDurationSeconds +
                blockedDeliveryBias -
                agingBonus;

            return new CostEvaluation(
                true,
                total,
                amrToPalletEta: amrToPallet,
                attachTime: pallet.AttachDurationSeconds,
                loadingQueueEta: loadingQueueEta,
                palletToLoadingEta: palletToLoading,
                loadTime: pallet.LoadDurationSeconds,
                loadingToWorkstationEta: loadingToWorkstation,
                detachTime: pallet.DetachDurationSeconds,
                blockedDeliveryBias: blockedDeliveryBias,
                agingBonus: agingBonus);
        }

        public CostEvaluation Evaluate(TaskPlanningAmr amr, PalletRemovalPlanningTask task)
        {
            if (amr == null || task?.Pallet == null || task.Pallet.CurrentNode == null || task.Pallet.ParkingNode == null)
                return CostEvaluation.Infeasible;

            if (task.Pallet.Status != PalletStatus.AwaitingRemoval)
                return CostEvaluation.Infeasible;

            var amrNode = _distances.NearestNode(amr.transform.position);
            if (amrNode == null)
                return CostEvaluation.Infeasible;

            var amrToPallet = Eta(amrNode, task.Pallet.CurrentNode);
            var palletToParking = Eta(task.Pallet.CurrentNode, task.Pallet.ParkingNode);
            if (!Finite(amrToPallet) || !Finite(palletToParking))
                return CostEvaluation.Infeasible;

            var baseTotal =
                _weights.amrToPalletEta * amrToPallet +
                _weights.attachTime * task.Pallet.AttachDurationSeconds +
                _weights.palletToParkingEta * palletToParking +
                _weights.detachTime * task.Pallet.DetachDurationSeconds;
            var multiplier = RemovalBlocksPendingDelivery(task)
                ? Mathf.Max(0f, _weights.removalBlocksPendingDeliveryMultiplier)
                : 1.0;
            var agingBonus = AgingBonus(task);
            var total = baseTotal * multiplier - agingBonus;

            return new CostEvaluation(
                true,
                total,
                amrToPalletEta: amrToPallet,
                attachTime: task.Pallet.AttachDurationSeconds,
                detachTime: task.Pallet.DetachDurationSeconds,
                palletToParkingEta: palletToParking,
                agingBonus: agingBonus,
                removalPriorityMultiplier: multiplier);
        }

        private double Eta(MapfNode from, MapfNode to)
        {
            return _distances.Distance(from, to) / _amrSpeed;
        }

        private static double EstimateLoadingQueueEta(PalletLoadingPoint loadingPoint)
        {
            var eta = 0.0;
            if (loadingPoint.ReservedFor != null)
                eta += loadingPoint.ReservedFor.LoadDurationSeconds;

            foreach (var pallet in loadingPoint.QueuedPallets)
                eta += pallet.LoadDurationSeconds;

            return eta;
        }

        private double AgingBonus(ITaskPlanningTask task)
        {
            var waitingTime = System.Math.Max(0.0, _now - task.EnqueuedTime);
            return System.Math.Min(_weights.maxAgingBonus, waitingTime * _weights.agingWeight);
        }

        private bool RemovalBlocksPendingDelivery(PalletRemovalPlanningTask removal)
        {
            if (removal.SourceWorkstation == null)
                return false;

            return _pendingTasks
                .OfType<DeliveryPlanningTask>()
                .Any(delivery => delivery.Workstation == removal.SourceWorkstation);
        }

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
