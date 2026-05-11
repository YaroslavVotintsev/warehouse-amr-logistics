using Mapf.Authoring;

namespace TaskPlanning
{
    public readonly struct AmrFutureAvailability
    {
        public readonly bool IsValid;
        public readonly TaskPlanningAmr Amr;
        public readonly ITaskPlanningTask ActiveTask;
        public readonly MapfNode FinishNode;
        public readonly double PriorAssignmentEta;
        public readonly bool IsInterruptible;

        public AmrFutureAvailability(
            TaskPlanningAmr amr,
            ITaskPlanningTask activeTask,
            MapfNode finishNode,
            double priorAssignmentEta,
            bool isInterruptible)
        {
            IsValid = amr != null && activeTask != null && finishNode != null;
            Amr = amr;
            ActiveTask = activeTask;
            FinishNode = finishNode;
            PriorAssignmentEta = priorAssignmentEta;
            IsInterruptible = isInterruptible;
        }
    }
}
