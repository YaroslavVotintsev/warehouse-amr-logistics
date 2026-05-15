using System;
using System.Collections.Generic;

namespace TaskPlanning
{
    public sealed class TaskPlanningDispatchCycleTrace
    {
        public TaskPlanningDispatchCycleTrace(
            int cycleNumber,
            float time,
            TaskPlanningAlgorithmType algorithm,
            TaskPlanningFutureHandlingMode futureHandling,
            IReadOnlyList<ITaskPlanningTask> pendingTasks,
            IReadOnlyList<TaskPlanningAmr> freeAmrs,
            IReadOnlyList<TaskPlanningAmr> activeInterruptibleAmrs,
            IReadOnlyList<SoftReassignmentOption> softReassignmentOptions,
            IReadOnlyList<AmrFutureAvailability> futureAvailabilities,
            IReadOnlyList<DispatchCandidate> candidates,
            IReadOnlyList<DispatchAssignment> selectedAssignments)
        {
            CycleNumber = cycleNumber;
            Time = time;
            Algorithm = algorithm;
            FutureHandling = futureHandling;
            PendingTasks = pendingTasks ?? Array.Empty<ITaskPlanningTask>();
            FreeAmrs = freeAmrs ?? Array.Empty<TaskPlanningAmr>();
            ActiveInterruptibleAmrs = activeInterruptibleAmrs ?? Array.Empty<TaskPlanningAmr>();
            SoftReassignmentOptions = softReassignmentOptions ?? Array.Empty<SoftReassignmentOption>();
            FutureAvailabilities = futureAvailabilities ?? Array.Empty<AmrFutureAvailability>();
            Candidates = candidates ?? Array.Empty<DispatchCandidate>();
            SelectedAssignments = selectedAssignments ?? Array.Empty<DispatchAssignment>();
        }

        public int CycleNumber { get; }
        public float Time { get; }
        public TaskPlanningAlgorithmType Algorithm { get; }
        public TaskPlanningFutureHandlingMode FutureHandling { get; }
        public IReadOnlyList<ITaskPlanningTask> PendingTasks { get; }
        public IReadOnlyList<TaskPlanningAmr> FreeAmrs { get; }
        public IReadOnlyList<TaskPlanningAmr> ActiveInterruptibleAmrs { get; }
        public IReadOnlyList<SoftReassignmentOption> SoftReassignmentOptions { get; }
        public IReadOnlyList<AmrFutureAvailability> FutureAvailabilities { get; }
        public IReadOnlyList<DispatchCandidate> Candidates { get; }
        public IReadOnlyList<DispatchAssignment> SelectedAssignments { get; }
    }
}
