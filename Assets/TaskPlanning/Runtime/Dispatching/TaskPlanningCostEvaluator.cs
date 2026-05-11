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
            _weights = weights ?? new TaskPlanningCostWeights();
            _amrSpeed = Mathf.Max(0.0001f, amrSpeed);
            _pendingTasks = pendingTasks ?? System.Array.Empty<ITaskPlanningTask>();
            _now = now;
        }

        private DeliveryTaskCostWeights DeliveryWeights => _weights.delivery ??= new DeliveryTaskCostWeights();
        private RemovalTaskCostWeights RemovalWeights => _weights.removal ??= new RemovalTaskCostWeights();

        public CostEvaluation Evaluate(TaskPlanningAmr amr, DeliveryPlanningTask task, PalletLoadingPoint loadingPoint)
        {
            var startNode = NearestNode(amr);
            return EvaluateDelivery(startNode, task, loadingPoint, false, null, 0);
        }

        public CostEvaluation EvaluateActiveAssignment(TaskPlanningAmr amr, DeliveryPlanningTask task, PalletLoadingPoint loadingPoint)
        {
            var startNode = NearestNode(amr);
            return EvaluateDelivery(startNode, task, loadingPoint, true, task?.Pallet, 0);
        }

        public CostEvaluation EvaluateFrom(
            MapfNode startNode,
            DeliveryPlanningTask task,
            PalletLoadingPoint loadingPoint,
            double priorAssignmentEta)
        {
            return EvaluateDelivery(startNode, task, loadingPoint, false, null, priorAssignmentEta);
        }

        private CostEvaluation EvaluateDelivery(
            MapfNode startNode,
            DeliveryPlanningTask task,
            PalletLoadingPoint loadingPoint,
            bool allowReservedPallet,
            PalletMarker ignoredQueuedPallet,
            double priorAssignmentEta)
        {
            if (startNode == null || task?.Pallet == null || task.Workstation == null || loadingPoint == null || !Finite(priorAssignmentEta))
                return CostEvaluation.Infeasible;

            var pallet = task.Pallet;
            if (!IsAvailableForDelivery(pallet, allowReservedPallet) ||
                pallet.CurrentNode == null ||
                loadingPoint.Node == null ||
                task.Workstation.Node == null)
                return CostEvaluation.Infeasible;

            if (!loadingPoint.Accepts(pallet) || !task.Workstation.Accepts(pallet))
                return CostEvaluation.Infeasible;

            var amrToPallet = Eta(startNode, pallet.CurrentNode);
            var palletToLoading = Eta(pallet.CurrentNode, loadingPoint.Node);
            var loadingToWorkstation = Eta(loadingPoint.Node, task.Workstation.Node);
            if (!Finite(amrToPallet) || !Finite(palletToLoading) || !Finite(loadingToWorkstation))
                return CostEvaluation.Infeasible;

            var weights = DeliveryWeights;
            var loadingQueueEta = EstimateLoadingQueueEta(loadingPoint, ignoredQueuedPallet);
            var blockedDeliveryBias = task.Workstation.HasBlockingPalletFor(pallet)
                ? weights.blockedDeliveryBias
                : 0.0;
            var agingBonus = AgingBonus(task, weights.agingWeight, weights.maxAgingBonus);
            var total =
                weights.priorAssignmentEta * priorAssignmentEta +
                weights.amrToPalletEta * amrToPallet +
                weights.attachTime * pallet.AttachDurationSeconds +
                weights.loadingQueueEta * loadingQueueEta +
                weights.palletToLoadingEta * palletToLoading +
                weights.loadTime * pallet.LoadDurationSeconds +
                weights.loadingToWorkstationEta * loadingToWorkstation +
                weights.detachTime * pallet.DetachDurationSeconds +
                blockedDeliveryBias -
                agingBonus;

            return new CostEvaluation(
                true,
                total,
                priorAssignmentEta: priorAssignmentEta,
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
            var startNode = NearestNode(amr);
            return EvaluateRemoval(startNode, task, false, 0);
        }

        public CostEvaluation EvaluateActiveAssignment(TaskPlanningAmr amr, PalletRemovalPlanningTask task)
        {
            var startNode = NearestNode(amr);
            return EvaluateRemoval(startNode, task, true, 0);
        }

        public CostEvaluation EvaluateFrom(
            MapfNode startNode,
            PalletRemovalPlanningTask task,
            double priorAssignmentEta)
        {
            return EvaluateRemoval(startNode, task, false, priorAssignmentEta);
        }

        private CostEvaluation EvaluateRemoval(
            MapfNode startNode,
            PalletRemovalPlanningTask task,
            bool allowReservedPallet,
            double priorAssignmentEta)
        {
            if (startNode == null || task?.Pallet == null || task.Pallet.CurrentNode == null || task.Pallet.ParkingNode == null || !Finite(priorAssignmentEta))
                return CostEvaluation.Infeasible;

            if (!IsAvailableForRemoval(task.Pallet, allowReservedPallet))
                return CostEvaluation.Infeasible;

            var amrToPallet = Eta(startNode, task.Pallet.CurrentNode);
            var palletToParking = Eta(task.Pallet.CurrentNode, task.Pallet.ParkingNode);
            if (!Finite(amrToPallet) || !Finite(palletToParking))
                return CostEvaluation.Infeasible;

            var weights = RemovalWeights;
            var operationTotal =
                weights.amrToPalletEta * amrToPallet +
                weights.attachTime * task.Pallet.AttachDurationSeconds +
                weights.palletToParkingEta * palletToParking +
                weights.detachTime * task.Pallet.DetachDurationSeconds;
            var multiplier = RemovalBlocksPendingDelivery(task)
                ? Mathf.Max(0f, weights.blocksPendingDeliveryMultiplier)
                : 1.0;
            var agingBonus = AgingBonus(task, weights.agingWeight, weights.maxAgingBonus);
            var total =
                weights.priorAssignmentEta * priorAssignmentEta +
                operationTotal * multiplier -
                agingBonus;

            return new CostEvaluation(
                true,
                total,
                priorAssignmentEta: priorAssignmentEta,
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

        private MapfNode NearestNode(TaskPlanningAmr amr)
        {
            return amr != null ? _distances.NearestNode(amr.transform.position) : null;
        }

        private static bool IsAvailableForDelivery(PalletMarker pallet, bool allowReservedPallet)
        {
            return pallet.IsAvailable ||
                (allowReservedPallet && (pallet.Status == PalletStatus.Reserved || pallet.Status == PalletStatus.Attaching));
        }

        private static bool IsAvailableForRemoval(PalletMarker pallet, bool allowReservedPallet)
        {
            return pallet.Status == PalletStatus.AwaitingRemoval ||
                (allowReservedPallet && (pallet.Status == PalletStatus.Reserved || pallet.Status == PalletStatus.Attaching));
        }

        private static double EstimateLoadingQueueEta(PalletLoadingPoint loadingPoint, PalletMarker ignoredPallet)
        {
            var eta = 0.0;
            if (loadingPoint.ReservedFor != null && loadingPoint.ReservedFor != ignoredPallet)
                eta += loadingPoint.ReservedFor.LoadDurationSeconds;

            foreach (var pallet in loadingPoint.QueuedPallets)
            {
                if (pallet == ignoredPallet)
                    continue;

                eta += pallet.LoadDurationSeconds;
            }

            return eta;
        }

        private double AgingBonus(ITaskPlanningTask task, float agingWeight, float maxAgingBonus)
        {
            var waitingTime = System.Math.Max(0.0, _now - task.EnqueuedTime);
            return System.Math.Min(maxAgingBonus, waitingTime * agingWeight);
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
