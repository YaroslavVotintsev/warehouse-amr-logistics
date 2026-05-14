using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace TaskPlanning
{
    public sealed class TaskPlanningScenario
    {
        public TaskPlanningScenario(
            string name,
            IReadOnlyList<TaskPlanningScenarioNode> nodes,
            IReadOnlyList<TaskPlanningScenarioEdge> edges,
            IReadOnlyList<TaskPlanningScenarioAmr> amrs,
            IReadOnlyList<TaskPlanningScenarioPallet> pallets,
            IReadOnlyList<TaskPlanningScenarioLoadingPoint> loadingPoints,
            IReadOnlyList<TaskPlanningScenarioWorkstation> workstations,
            IReadOnlyList<ScheduledMesTask> scheduledTasks)
        {
            Name = name;
            Nodes = nodes ?? System.Array.Empty<TaskPlanningScenarioNode>();
            Edges = edges ?? System.Array.Empty<TaskPlanningScenarioEdge>();
            Amrs = amrs ?? System.Array.Empty<TaskPlanningScenarioAmr>();
            Pallets = pallets ?? System.Array.Empty<TaskPlanningScenarioPallet>();
            LoadingPoints = loadingPoints ?? System.Array.Empty<TaskPlanningScenarioLoadingPoint>();
            Workstations = workstations ?? System.Array.Empty<TaskPlanningScenarioWorkstation>();
            ScheduledTasks = scheduledTasks ?? System.Array.Empty<ScheduledMesTask>();
        }

        public string Name { get; }
        public IReadOnlyList<TaskPlanningScenarioNode> Nodes { get; }
        public IReadOnlyList<TaskPlanningScenarioEdge> Edges { get; }
        public IReadOnlyList<TaskPlanningScenarioAmr> Amrs { get; }
        public IReadOnlyList<TaskPlanningScenarioPallet> Pallets { get; }
        public IReadOnlyList<TaskPlanningScenarioLoadingPoint> LoadingPoints { get; }
        public IReadOnlyList<TaskPlanningScenarioWorkstation> Workstations { get; }
        public IReadOnlyList<ScheduledMesTask> ScheduledTasks { get; }

        public IReadOnlyList<string> PalletIds => Pallets.Select(pallet => pallet.PalletId).ToArray();
    }

    public readonly struct TaskPlanningScenarioNode
    {
        public readonly string NodeId;
        public readonly Vector2 Position;

        public TaskPlanningScenarioNode(string nodeId, Vector2 position)
        {
            NodeId = nodeId;
            Position = position;
        }
    }

    public readonly struct TaskPlanningScenarioEdge
    {
        public readonly string ANodeId;
        public readonly string BNodeId;

        public TaskPlanningScenarioEdge(string aNodeId, string bNodeId)
        {
            ANodeId = aNodeId;
            BNodeId = bNodeId;
        }
    }

    public readonly struct TaskPlanningScenarioAmr
    {
        public readonly string AmrId;
        public readonly int AgentId;
        public readonly string StartNodeId;

        public TaskPlanningScenarioAmr(string amrId, int agentId, string startNodeId)
        {
            AmrId = amrId;
            AgentId = agentId;
            StartNodeId = startNodeId;
        }
    }

    public readonly struct TaskPlanningScenarioPallet
    {
        public readonly string PalletId;
        public readonly string CurrentNodeId;
        public readonly string ParkingNodeId;
        public readonly float AttachSeconds;
        public readonly float DetachSeconds;
        public readonly float LoadSeconds;
        public readonly float UnloadSeconds;

        public TaskPlanningScenarioPallet(
            string palletId,
            string currentNodeId,
            string parkingNodeId,
            float attachSeconds = 1f,
            float detachSeconds = 1f,
            float loadSeconds = 3f,
            float unloadSeconds = 3f)
        {
            PalletId = palletId;
            CurrentNodeId = currentNodeId;
            ParkingNodeId = parkingNodeId;
            AttachSeconds = attachSeconds;
            DetachSeconds = detachSeconds;
            LoadSeconds = loadSeconds;
            UnloadSeconds = unloadSeconds;
        }
    }

    public readonly struct TaskPlanningScenarioLoadingPoint
    {
        public readonly string LoadingPointId;
        public readonly string NodeId;
        public readonly IReadOnlyList<string> AcceptedPalletIds;

        public TaskPlanningScenarioLoadingPoint(string loadingPointId, string nodeId, IReadOnlyList<string> acceptedPalletIds)
        {
            LoadingPointId = loadingPointId;
            NodeId = nodeId;
            AcceptedPalletIds = acceptedPalletIds ?? System.Array.Empty<string>();
        }
    }

    public readonly struct TaskPlanningScenarioWorkstation
    {
        public readonly string WorkstationId;
        public readonly string NodeId;
        public readonly IReadOnlyList<string> AcceptedPalletIds;

        public TaskPlanningScenarioWorkstation(string workstationId, string nodeId, IReadOnlyList<string> acceptedPalletIds)
        {
            WorkstationId = workstationId;
            NodeId = nodeId;
            AcceptedPalletIds = acceptedPalletIds ?? System.Array.Empty<string>();
        }
    }
}
