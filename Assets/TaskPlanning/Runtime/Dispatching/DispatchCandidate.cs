using Mapf.Authoring;

namespace TaskPlanning
{
    public readonly struct DispatchCandidate
    {
        public readonly bool IsValid;
        public readonly DispatchAvailability Availability;
        public readonly ITaskPlanningTask Task;
        public readonly PalletMarker Pallet;
        public readonly PalletLoadingPoint LoadingPoint;
        public readonly WorkstationDeliveryPoint Workstation;
        public readonly MapfNode RemovalTargetNode;
        public readonly CostEvaluation Cost;

        public TaskPlanningAmr Amr => Availability.Amr;
        public double Score => Cost.TotalCost;

        public DispatchCandidate(
            DispatchAvailability availability,
            ITaskPlanningTask task,
            PalletMarker pallet,
            PalletLoadingPoint loadingPoint,
            WorkstationDeliveryPoint workstation,
            MapfNode removalTargetNode,
            CostEvaluation cost)
        {
            IsValid = availability.IsValid && task != null && pallet != null && cost.IsFeasible;
            Availability = availability;
            Task = task;
            Pallet = pallet;
            LoadingPoint = loadingPoint;
            Workstation = workstation;
            RemovalTargetNode = removalTargetNode;
            Cost = cost;
        }

        public DispatchAssignment ToAssignment()
        {
            return new DispatchAssignment(
                Amr,
                Task,
                Pallet,
                LoadingPoint,
                Workstation,
                RemovalTargetNode,
                Cost);
        }
    }
}
