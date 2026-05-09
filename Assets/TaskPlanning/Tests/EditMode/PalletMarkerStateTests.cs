using Mapf.Authoring;
using NUnit.Framework;
using UnityEngine;

namespace TaskPlanning.Tests
{
    public sealed class PalletMarkerStateTests
    {
        [Test]
        public void PalletMovesThroughExpectedLifecycleStates()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("Kit-A");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("AMR");
            var workstationNode = TaskPlanningTestHelpers.CreateNode("Workstation", new Vector2(2f, 0f));

            try
            {
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Available));
                Assert.That(pallet.IsAvailable, Is.True);
                Assert.That(pallet.IsLoaded, Is.False);

                Assert.That(pallet.TryReserve(), Is.True);
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Reserved));
                Assert.That(pallet.TryReserve(), Is.False);

                pallet.MarkAttaching();
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Attaching));

                pallet.AttachTo(amr);
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Attached));
                Assert.That(pallet.CurrentNode, Is.Null);
                Assert.That(amr.AttachedPallet, Is.SameAs(pallet));

                pallet.MarkLoading();
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Loading));
                Assert.That(pallet.IsLoaded, Is.False);

                pallet.MarkLoaded();
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Loaded));
                Assert.That(pallet.IsLoaded, Is.True);

                pallet.MarkDetaching();
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Detaching));

                pallet.DetachAt(workstationNode, PalletStatus.Unloading);
                Assert.That(pallet.CurrentNode, Is.SameAs(workstationNode));
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Unloading));
                Assert.That(pallet.IsLoaded, Is.True);

                pallet.MarkUnloadedAvailable();
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Available));
                Assert.That(pallet.IsAvailable, Is.True);
                Assert.That(pallet.IsLoaded, Is.False);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(workstationNode.gameObject);
                TaskPlanningTestHelpers.Destroy(amr.gameObject);
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
            }
        }

        [Test]
        public void TryReserveForRemovalOnlyWorksWhenAwaitingRemoval()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("RemovalOnly");

            try
            {
                Assert.That(pallet.TryReserveForRemoval(), Is.False);

                pallet.MarkAwaitingRemoval();

                Assert.That(pallet.TryReserveForRemoval(), Is.True);
                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Reserved));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
            }
        }

        [Test]
        public void ReleasePendingReservationRestoresDeliveryReservation()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("DeliveryReservation");

            try
            {
                Assert.That(pallet.TryReserve(), Is.True);

                pallet.ReleasePendingReservation(PalletStatus.Available);

                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Available));
                Assert.That(pallet.IsAvailable, Is.True);
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
            }
        }

        [Test]
        public void ReleasePendingReservationRestoresRemovalReservation()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("RemovalReservation");

            try
            {
                pallet.MarkAwaitingRemoval();
                Assert.That(pallet.TryReserveForRemoval(), Is.True);

                pallet.ReleasePendingReservation(PalletStatus.AwaitingRemoval);

                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.AwaitingRemoval));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
            }
        }

        [Test]
        public void ReleasePendingReservationDoesNotChangeAttachedPallet()
        {
            var pallet = TaskPlanningTestHelpers.CreatePallet("AttachedReservation");
            var amr = TaskPlanningTestHelpers.CreateComponent<TaskPlanningAmr>("AttachedReservationAmr");

            try
            {
                Assert.That(pallet.TryReserve(), Is.True);
                pallet.AttachTo(amr);

                pallet.ReleasePendingReservation(PalletStatus.Available);

                Assert.That(pallet.Status, Is.EqualTo(PalletStatus.Attached));
                Assert.That(amr.AttachedPallet, Is.SameAs(pallet));
            }
            finally
            {
                TaskPlanningTestHelpers.Destroy(pallet.gameObject);
                TaskPlanningTestHelpers.Destroy(amr.gameObject);
            }
        }
    }
}
