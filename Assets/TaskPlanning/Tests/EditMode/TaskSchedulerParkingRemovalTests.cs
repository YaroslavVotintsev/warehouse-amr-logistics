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

        private static List<ITaskPlanningTask> PendingTasks(TaskScheduler scheduler)
        {
            return TaskPlanningTestHelpers.GetField<List<ITaskPlanningTask>>(scheduler, "_pendingTasks");
        }
    }
}
