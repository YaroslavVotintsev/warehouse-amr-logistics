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
    }
}
