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
        public void LoadingPointResolverFindsSingleAcceptedPoint()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("ResolverSingle_Pallet");
            var other = TaskPlanningTestHelpers.CreatePallet("ResolverSingle_Other");
            var nodeA = TaskPlanningTestHelpers.CreateNode("ResolverSingle_NodeA", Vector2.zero);
            var nodeB = TaskPlanningTestHelpers.CreateNode("ResolverSingle_NodeB", Vector2.right);
            var loadingA = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverSingle_A", nodeA, other);
            var loadingB = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverSingle_B", nodeB, pallet);

            try
            {
                var resolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, new[] { loadingA, loadingB });

                Assert.That(resolution.IsResolved, Is.True);
                Assert.That(resolution.LoadingPoint, Is.SameAs(loadingB));
                Assert.That(resolution.Status, Is.EqualTo(PalletLoadingPointResolutionStatus.Resolved));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    loadingB.gameObject,
                    loadingA.gameObject,
                    nodeB.gameObject,
                    nodeA.gameObject,
                    other.gameObject,
                    pallet.gameObject);
            }
        }

        [Test]
        public void LoadingPointResolverRejectsMissingAcceptedPoint()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("ResolverMissing_Pallet");
            var other = TaskPlanningTestHelpers.CreatePallet("ResolverMissing_Other");
            var node = TaskPlanningTestHelpers.CreateNode("ResolverMissing_Node", Vector2.zero);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverMissing_Load", node, other);

            try
            {
                var resolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, new[] { loadingPoint });

                Assert.That(resolution.IsResolved, Is.False);
                Assert.That(resolution.Status, Is.EqualTo(PalletLoadingPointResolutionStatus.NoAcceptingLoadingPoint));
                Assert.That(resolution.Message, Does.Contain(pallet.KitId));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    loadingPoint.gameObject,
                    node.gameObject,
                    other.gameObject,
                    pallet.gameObject);
            }
        }

        [Test]
        public void LoadingPointResolverRejectsMultipleAcceptedPoints()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("ResolverMultiple_Pallet");
            var nodeA = TaskPlanningTestHelpers.CreateNode("ResolverMultiple_NodeA", Vector2.zero);
            var nodeB = TaskPlanningTestHelpers.CreateNode("ResolverMultiple_NodeB", Vector2.right);
            var loadingA = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverMultiple_A", nodeA, pallet);
            var loadingB = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverMultiple_B", nodeB, pallet);

            try
            {
                var resolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, new[] { loadingA, loadingB });

                Assert.That(resolution.IsResolved, Is.False);
                Assert.That(resolution.Status, Is.EqualTo(PalletLoadingPointResolutionStatus.MultipleAcceptingLoadingPoints));
                Assert.That(resolution.Message, Does.Contain(loadingA.LoadingPointId));
                Assert.That(resolution.Message, Does.Contain(loadingB.LoadingPointId));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    loadingB.gameObject,
                    loadingA.gameObject,
                    nodeB.gameObject,
                    nodeA.gameObject,
                    pallet.gameObject);
            }
        }

        [Test]
        public void LoadingPointResolverRejectsAcceptedPointWithoutNode()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("ResolverNoNode_Pallet");
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("ResolverNoNode_Load", null, pallet);

            try
            {
                var resolution = PalletLoadingPoint.ResolveAcceptedLoadingPoint(pallet, new[] { loadingPoint });

                Assert.That(resolution.IsResolved, Is.False);
                Assert.That(resolution.Status, Is.EqualTo(PalletLoadingPointResolutionStatus.MissingNode));
                Assert.That(resolution.LoadingPoint, Is.SameAs(loadingPoint));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    loadingPoint.gameObject,
                    pallet.gameObject);
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
