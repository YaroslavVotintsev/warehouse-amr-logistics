using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskPlanningQueueTests
    {
        [Test]
        public void LoadingPointReservesQueuedPalletsInFifoOrder()
        {
            var a = TaskPlanningTestHelpers.CreatePallet("A");
            var b = TaskPlanningTestHelpers.CreatePallet("B");
            var node = TaskPlanningTestHelpers.CreateNode("LoadNode", Vector2.zero);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Load", node, a, b);

            try
            {
                Assert.That(loadingPoint.Enqueue(a), Is.True);
                Assert.That(loadingPoint.Enqueue(b), Is.True);
                Assert.That(loadingPoint.QueueLength, Is.EqualTo(2));
                Assert.That(loadingPoint.CanReserveNext(b), Is.False);

                Assert.That(loadingPoint.TryReserveNext(a), Is.True);
                Assert.That(loadingPoint.ReservedFor, Is.SameAs(a));
                Assert.That(loadingPoint.QueueLength, Is.EqualTo(1));

                loadingPoint.ReleaseReservation(a);

                Assert.That(loadingPoint.TryReserveNext(b), Is.True);
                Assert.That(loadingPoint.ReservedFor, Is.SameAs(b));
                Assert.That(loadingPoint.QueueLength, Is.EqualTo(0));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(loadingPoint.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(b.gameObject);
                TaskPlanningTestHelpers.Destroy(a.gameObject);
            }
        }

        [Test]
        public void WorkstationReservesQueuedPalletsInFifoOrder()
        {
            var a = TaskPlanningTestHelpers.CreatePallet("A");
            var b = TaskPlanningTestHelpers.CreatePallet("B");
            var node = TaskPlanningTestHelpers.CreateNode("WorkstationNode", Vector2.zero);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("Workstation", node, a, b);

            try
            {
                a.transform.position = new Vector3(2f, 0f, 0f);
                b.transform.position = new Vector3(3f, 0f, 0f);

                Assert.That(workstation.Enqueue(a), Is.True);
                Assert.That(workstation.Enqueue(b), Is.True);
                Assert.That(workstation.QueueLength, Is.EqualTo(2));
                Assert.That(workstation.CanReserveNext(b), Is.False);

                Assert.That(workstation.TryReserveNext(a), Is.True);
                Assert.That(workstation.ReservedFor, Is.SameAs(a));
                Assert.That(workstation.QueueLength, Is.EqualTo(1));

                workstation.ReleaseReservation(a);

                Assert.That(workstation.TryReserveNext(b), Is.True);
                Assert.That(workstation.ReservedFor, Is.SameAs(b));
                Assert.That(workstation.QueueLength, Is.EqualTo(0));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstation.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(b.gameObject);
                TaskPlanningTestHelpers.Destroy(a.gameObject);
            }
        }

        [Test]
        public void DuplicateEnqueueDoesNotIncreaseQueueLength()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("A");
            var node = TaskPlanningTestHelpers.CreateNode("LoadNode", Vector2.zero);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Load", node, pallet);

            try
            {
                Assert.That(loadingPoint.Enqueue(pallet), Is.True);
                Assert.That(loadingPoint.Enqueue(pallet), Is.True);

                Assert.That(loadingPoint.QueueLength, Is.EqualTo(1));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(loadingPoint.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
            }
        }

        [Test]
        public void ReleasingReservationWithWrongPalletDoesNotClearCurrentReservation()
        {
            var reserved = TaskPlanningTestHelpers.CreatePallet("Reserved");
            var wrong = TaskPlanningTestHelpers.CreatePallet("Wrong");
            var node = TaskPlanningTestHelpers.CreateNode("LoadNode", Vector2.zero);
            var loadingPoint = TaskPlanningTestHelpers.CreateLoadingPoint("Load", node, reserved, wrong);

            try
            {
                Assert.That(loadingPoint.Enqueue(reserved), Is.True);
                Assert.That(loadingPoint.TryReserveNext(reserved), Is.True);

                loadingPoint.ReleaseReservation(wrong);

                Assert.That(loadingPoint.ReservedFor, Is.SameAs(reserved));

                loadingPoint.ReleaseReservation(reserved);

                Assert.That(loadingPoint.ReservedFor, Is.Null);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(loadingPoint.gameObject);
                TaskPlanningTestHelpers.Destroy(node.gameObject);
                TaskPlanningTestHelpers.Destroy(wrong.gameObject);
                TaskPlanningTestHelpers.Destroy(reserved.gameObject);
            }
        }
    }
}
