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
        public readonly DispatchAvailabilityType AvailabilityType;

        public bool IsImmediate => AvailabilityType != DispatchAvailabilityType.Future;
        public bool IsSoftReassignment => AvailabilityType == DispatchAvailabilityType.SoftReassignment;

        public DispatchAvailability(
            TaskPlanningAmr amr,
            MapfNode startNode,
            double availableAt,
            double priorAssignmentEta,
            bool isImmediate)
            : this(
                amr,
                startNode,
                availableAt,
                priorAssignmentEta,
                isImmediate ? DispatchAvailabilityType.Immediate : DispatchAvailabilityType.Future)
        {
        }

        public DispatchAvailability(
            TaskPlanningAmr amr,
            MapfNode startNode,
            double availableAt,
            double priorAssignmentEta,
            DispatchAvailabilityType availabilityType)
        {
            IsValid = amr != null && startNode != null;
            Amr = amr;
            StartNode = startNode;
            AvailableAt = availableAt;
            PriorAssignmentEta = priorAssignmentEta;
            AvailabilityType = availabilityType;
        }
    }
}
