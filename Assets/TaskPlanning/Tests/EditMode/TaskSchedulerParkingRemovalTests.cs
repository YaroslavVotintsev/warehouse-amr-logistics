using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class TaskSchedulerParkingRemovalTests
    {
        [Test]
        public void EnqueueRemovalTaskSkipsPalletAlreadyAtParkingNode()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("ParkingSkip_Scheduler");
            var node = TaskPlanningTestHelpers.CreateNode("ParkingSkip_Node", Vector2.zero);
            var pallet = TaskPlanningTestHelpers.CreatePallet("ParkingSkip_Pallet", node);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("ParkingSkip_Workstation", node, pallet);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(pallet, node);
                pallet.MarkAwaitingRemoval();

                TaskPlanningTestHelpers.InvokePrivate(scheduler, "EnqueueRemovalTask", pallet, workstation);
                var pending = PendingTasks(scheduler);

                Assert.That(pending, Is.Empty);
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Available));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstation.gameObject, pallet.gameObject, node.gameObject, scheduler.gameObject);
            }
        }

        [Test]
        public void AlreadyParkedRemovalTaskCompletesWithoutAmr()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("ParkingComplete_Scheduler");
            var node = TaskPlanningTestHelpers.CreateNode("ParkingComplete_Node", Vector2.zero);
            var pallet = TaskPlanningTestHelpers.CreatePallet("ParkingComplete_Pallet", node);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("ParkingComplete_Workstation", node, pallet);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(pallet, node);
                pallet.MarkAwaitingRemoval();
                var pending = PendingTasks(scheduler);
                pending.Add(new PalletRemovalPlanningTask("R-Parked", pallet, workstation, 0f));

                TaskPlanningTestHelpers.InvokePrivate(scheduler, "CompleteAlreadyParkedRemovalTasks");

                Assert.That(pending, Is.Empty);
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Available));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstation.gameObject, pallet.gameObject, node.gameObject, scheduler.gameObject);
            }
        }

        [Test]
        public void EnqueueRemovalTaskAddsTaskWhenParkingNodeDiffersFromWorkstation()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("ParkingQueue_Scheduler");
            var workstationNode = TaskPlanningTestHelpers.CreateNode("ParkingQueue_WorkstationNode", Vector2.zero);
            var parkingNode = TaskPlanningTestHelpers.CreateNode("ParkingQueue_ParkingNode", new Vector2(2f, 0f));
            var pallet = TaskPlanningTestHelpers.CreatePallet("ParkingQueue_Pallet", workstationNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("ParkingQueue_Workstation", workstationNode, pallet);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(pallet, parkingNode);
                pallet.MarkAwaitingRemoval();

                TaskPlanningTestHelpers.InvokePrivate(scheduler, "EnqueueRemovalTask", pallet, workstation);
                var pending = PendingTasks(scheduler);

                Assert.That(pending, Has.Count.EqualTo(1));
                Assert.That(pending[0], Is.TypeOf<PalletRemovalPlanningTask>());
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.AwaitingRemoval));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    pallet.gameObject,
                    parkingNode.gameObject,
                    workstationNode.gameObject,
                    scheduler.gameObject);
            }
        }

        [Test]
        public void AwaitingRemovalPalletBlocksWorkstationUntilMovedToParking()
        {
            var workstationNode = TaskPlanningTestHelpers.CreateNode("ParkingBlock_WorkstationNode", Vector2.zero);
            var parkingNode = TaskPlanningTestHelpers.CreateNode("ParkingBlock_ParkingNode", new Vector2(2f, 0f));
            var incoming = TaskPlanningTestHelpers.CreatePallet("ParkingBlock_Incoming");
            var blocking = TaskPlanningTestHelpers.CreatePallet("ParkingBlock_Blocking", workstationNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("ParkingBlock_Workstation", workstationNode, incoming);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(blocking, parkingNode);
                blocking.MarkAwaitingRemoval();

                Assert.That(workstation.Enqueue(incoming), Is.True);
                Assert.That(workstation.CanReserveNext(incoming), Is.False);

                blocking.DetachAt(parkingNode);
                blocking.MarkUnloadedAvailable();

                Assert.That(workstation.CanReserveNext(incoming), Is.True);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    blocking.gameObject,
                    incoming.gameObject,
                    parkingNode.gameObject,
                    workstationNode.gameObject);
            }
        }

        [Test]
        public void SchedulerDoesNotDispatchDeliveryWhileTargetWorkstationIsPhysicallyBlocked()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("BlockedDelivery_Scheduler");
            var workstationNode = TaskPlanningTestHelpers.CreateNode("BlockedDelivery_WorkstationNode", Vector2.zero);
            var parkingNode = TaskPlanningTestHelpers.CreateNode("BlockedDelivery_ParkingNode", new Vector2(2f, 0f));
            var incoming = TaskPlanningTestHelpers.CreatePallet("BlockedDelivery_Incoming");
            var blocking = TaskPlanningTestHelpers.CreatePallet("BlockedDelivery_Blocking", workstationNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("BlockedDelivery_Workstation", workstationNode, incoming);
            var delivery = new DeliveryPlanningTask("D-Blocked", incoming, workstation, 0f);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(blocking, parkingNode);
                blocking.MarkAwaitingRemoval();

                var blockedSnapshot = DispatchableTasks(scheduler, delivery);
                Assert.That(blockedSnapshot, Is.Empty);

                blocking.DetachAt(parkingNode);
                blocking.MarkUnloadedAvailable();

                var unblockedSnapshot = DispatchableTasks(scheduler, delivery);
                Assert.That(unblockedSnapshot.Count, Is.EqualTo(1));
                Assert.That(unblockedSnapshot[0], Is.SameAs(delivery));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    blocking.gameObject,
                    incoming.gameObject,
                    parkingNode.gameObject,
                    workstationNode.gameObject,
                    scheduler.gameObject);
            }
        }

        [Test]
        public void SchedulerKeepsRemovalDispatchableWhenDeliveryToSameWorkstationIsBlocked()
        {
            var scheduler = TaskPlanningTestHelpers.CreateComponent<TaskScheduler>("BlockedDeliveryRemoval_Scheduler");
            var workstationNode = TaskPlanningTestHelpers.CreateNode("BlockedDeliveryRemoval_WorkstationNode", Vector2.zero);
            var parkingNode = TaskPlanningTestHelpers.CreateNode("BlockedDeliveryRemoval_ParkingNode", new Vector2(2f, 0f));
            var incoming = TaskPlanningTestHelpers.CreatePallet("BlockedDeliveryRemoval_Incoming");
            var blocking = TaskPlanningTestHelpers.CreatePallet("BlockedDeliveryRemoval_Blocking", workstationNode);
            var workstation = TaskPlanningTestHelpers.CreateWorkstation("BlockedDeliveryRemoval_Workstation", workstationNode, incoming);
            var delivery = new DeliveryPlanningTask("D-Blocked", incoming, workstation, 0f);
            var removal = new PalletRemovalPlanningTask("R-Blocking", blocking, workstation, 0f);

            try
            {
                TaskPlanningTestHelpers.SetParkingNode(blocking, parkingNode);
                blocking.MarkAwaitingRemoval();

                var dispatchable = DispatchableTasks(scheduler, delivery, removal);

                Assert.That(dispatchable.Count, Is.EqualTo(1));
                Assert.That(dispatchable[0], Is.SameAs(removal));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(
                    workstation.gameObject,
                    blocking.gameObject,
                    incoming.gameObject,
                    parkingNode.gameObject,
                    workstationNode.gameObject,
                    scheduler.gameObject);
            }
        }

        private static List<ITaskPlanningTask> PendingTasks(TaskScheduler scheduler)
        {
            return TaskPlanningTestHelpers.GetField<List<ITaskPlanningTask>>(scheduler, "_pendingTasks");
        }

        private static IReadOnlyList<ITaskPlanningTask> DispatchableTasks(
            TaskScheduler scheduler,
            params ITaskPlanningTask[] tasks)
        {
            return (IReadOnlyList<ITaskPlanningTask>)TaskPlanningTestHelpers.InvokePrivate(
                scheduler,
                "BuildDispatchableTaskSnapshot",
                new object[] { tasks });
        }
    }
}
