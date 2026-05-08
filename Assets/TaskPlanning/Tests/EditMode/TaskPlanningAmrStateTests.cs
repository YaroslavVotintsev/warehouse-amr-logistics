using NUnit.Framework;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningAmrStateTests
    {
        [Test]
        public void AmrReserveRejectsWhileBusyAndAllowsAfterRelease()
        {
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("AMR");

            try
            {
                Assert.That(amr.IsBusy, Is.False);
                Assert.That(amr.TryReserve(), Is.True);
                Assert.That(amr.IsBusy, Is.True);
                Assert.That(amr.TryReserve(), Is.False);

                amr.Release();

                Assert.That(amr.IsBusy, Is.False);
                Assert.That(amr.TryReserve(), Is.True);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(amr.gameObject);
            }
        }
    }
}
