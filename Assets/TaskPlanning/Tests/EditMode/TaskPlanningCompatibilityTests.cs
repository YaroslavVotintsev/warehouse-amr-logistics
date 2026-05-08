using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningCompatibilityTests
    {
        [Test]
        public void LoadingPointAcceptsOnlyConfiguredPallets()
        {
            var accepted = TaskPlanningTestHelpers.CreatePallet("Accepted");
            var rejected = TaskPlanningTestHelpers.CreatePallet("Rejected");
            var node = TaskPlanningTestHelpers.CreateNode("LoadNode", Vector2.zero);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Load", node, accepted);

            try
            {
                Assert.That(loadingPoint.Accepts(accepted), Is.True);
                Assert.That(loadingPoint.Accepts(rejected), Is.False);
                Assert.That(loadingPoint.Accepts(null), Is.False);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(loadingPoint.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(rejected.gameObject);
                TaskPlanningTestHelpers.Destroy(accepted.gameObject);
            }
        }

        [Test]
        public void WorkstationAcceptsOnlyConfiguredPallets()
        {
            var accepted = TaskPlanningTestHelpers.CreatePallet("Accepted");
            var rejected = TaskPlanningTestHelpers.CreatePallet("Rejected");
            var node = TaskPlanningTestHelpers.CreateNode("WorkstationNode", Vector2.zero);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Workstation", node, accepted);

            try
            {
                Assert.That(workstation.Accepts(accepted), Is.True);
                Assert.That(workstation.Accepts(rejected), Is.False);
                Assert.That(workstation.Accepts(null), Is.False);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstation.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(rejected.gameObject);
                TaskPlanningTestHelpers.Destroy(accepted.gameObject);
            }
        }

        [Test]
        public void WorkstationCannotReserveNextWhileAnotherAvailablePalletOccupiesNode()
        {
            var workstationNode = TaskPlanningTestHelpers.CreateNode("WorkstationNode", Vector2.zero);
            var incoming = TaskPlanningTestHelpers.CreatePallet("Incoming");
            var blocking = TaskPlanningTestHelpers.CreatePallet("Blocking");
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Workstation", workstationNode, incoming);

            try
            {
                blocking.transform.position = workstationNode.transform.position;

                Assert.That(workstation.Enqueue(incoming), Is.True);
                Assert.That(workstation.CanReserveNext(incoming), Is.False);

                blocking.transform.position = new Vector3(2f, 0f, 0f);

                Assert.That(workstation.CanReserveNext(incoming), Is.True);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstation.gameObject);
                TaskPlanningTestHelpers.Destroy(blocking.gameObject);
                TaskPlanningTestHelpers.Destroy(incoming.gameObject);
                TaskPlanningTestHelpers.Destroy(workstationNode.gameObject);
            }
        }
    }
}
