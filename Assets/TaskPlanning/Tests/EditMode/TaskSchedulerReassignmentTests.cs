using NUnit.Framework;

namespace TaskPlanning.Tests
{
    public sealed class TaskSchedulerReassignmentTests
    {
        [Test]
        public void ReassignmentThresholdUsesPercentageImprovement()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("ReassignmentThreshold_Scheduler");

            try
            {
                TaskPlanningTestHelpers.SetField(scheduler, "reassignmentCostImprovementPercent", 10f);

                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 91.0), Is.False);
                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 90.0), Is.True);
                Assert.That(EnoughImprovement(scheduler, currentCost: 100.0, replacementCost: 101.0), Is.False);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(scheduler.gameObject);
            }
        }

        private static bool EnoughImprovement(TaskScheduler scheduler, double currentCost, double replacementCost)
        {
            return (bool)TaskPlanningTestHelpers.InvokePrivate(
                scheduler,
                "IsEnoughReassignmentImprovement",
                currentCost,
                replacementCost);
        }
    }
}
