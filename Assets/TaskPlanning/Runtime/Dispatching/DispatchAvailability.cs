using Mapf.Authoring;

namespace TaskPlanning
{
    public readonly struct DispatchAvailability
    {
        public readonly bool IsValid;
        public readonly TaskPlanningAmr Amr;
        public readonly MapfNode StartNode;
        public readonly double AvailableAt;
        public readonly double PriorAssignmentEta;
        public readonly bool IsImmediate;

        public DispatchAvailability(
            TaskPlanningAmr amr,
            MapfNode startNode,
            double availableAt,
            double priorAssignmentEta,
            bool isImmediate)
        {
            IsValid = amr != null && startNode != null;
            Amr = amr;
            StartNode = startNode;
            AvailableAt = availableAt;
            PriorAssignmentEta = priorAssignmentEta;
            IsImmediate = isImmediate;
        }
    }
}
