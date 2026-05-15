using System.Collections.Generic;
using System.IO;
using System.Linq;
using Mapf.Authoring;
using Mapf.UnityAdapter;
using UnityEngine;
using UnityEngine.Serialization;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace TaskPlanning
{
    public sealed class TaskPlanningScenarioSpawner : MonoBehaviour
    {
        public const string MesScheduledScenarioAssetFolder = "Assets/TaskPlanning/Scenarios";
        public const string MetricOutputFolder = "Assets/TaskPlanning/Scenarios/Metrics";
        private const string MapfRootName = "MAPF";
        private const string SchedulerRootName = "Scheduler";
        private const string MesRootName = "MES";

        [SerializeField] private TaskPlanningScenarioPreset preset = TaskPlanningScenarioPreset.FifoAssignmentTrap;
        [SerializeField] private bool spawnOnStart;
        [SerializeField] private bool clearChildrenBeforeSpawn = true;
        [FormerlySerializedAs("saveScenarioAssetInProject")]
        [SerializeField] private bool saveMesScheduledScenarioAssetInProject = true;
        [SerializeField] private Sprite agentSprite;
        [SerializeField] private Sprite palletSprite;
        [SerializeField] private Color[] amrColors = { Color.cyan, Color.yellow, Color.magenta, Color.green };
        [SerializeField] private Color palletColor = new(0.9f, 0.7f, 0.2f);

        private void Start()
        {
            if (spawnOnStart)
                Spawn();
        }

        public TaskPlanningScenarioPreset Preset => preset;

        [ContextMenu("Spawn Task Planning Scenario")]
        public void Spawn()
        {
            RemoveLegacySystemComponentsFromSpawner();

            if (clearChildrenBeforeSpawn)
                ClearChildren();

            var scenario = TaskPlanningScenarioLibrary.Get(preset);
            var nodeObjects = SpawnNodes(scenario);
            SpawnEdges(scenario, nodeObjects);
            var pallets = SpawnPallets(scenario, nodeObjects);
            var loadingPoints = SpawnLoadingPoints(scenario, nodeObjects, pallets);
            SpawnWorkstations(scenario, nodeObjects, pallets);
            var amrs = SpawnAmrs(scenario, nodeObjects);
            ConfigureSystemComponents(scenario, amrs, loadingPoints);
        }

        private Dictionary<string, MapfNode> SpawnNodes(TaskPlanningScenario scenario)
        {
            var root = CreateChildRoot("Nodes");
            var nodes = new Dictionary<string, MapfNode>();
            foreach (var node in scenario.Nodes)
            {
                var go = new GameObject(node.NodeId);
                go.transform.SetParent(root.transform, false);
                go.transform.position = new Vector3(node.Position.x, node.Position.y, 0f);

                var mapfNode = go.AddComponent<MapfNode>();
                mapfNode.Configure(node.NodeId);
                nodes[node.NodeId] = mapfNode;
            }

            return nodes;
        }

        private void SpawnEdges(TaskPlanningScenario scenario, IReadOnlyDictionary<string, MapfNode> nodes)
        {
            var root = CreateChildRoot("Edges");
            var usedEdges = new HashSet<string>();
            foreach (var edge in scenario.Edges)
            {
                if (!nodes.TryGetValue(edge.ANodeId, out var a) || !nodes.TryGetValue(edge.BNodeId, out var b))
                {
                    Debug.LogWarning($"Skipping edge '{edge.ANodeId}'-'{edge.BNodeId}' because one endpoint is missing.", this);
                    continue;
                }

                var edgeKey = string.CompareOrdinal(edge.ANodeId, edge.BNodeId) <= 0
                    ? $"{edge.ANodeId}|{edge.BNodeId}"
                    : $"{edge.BNodeId}|{edge.ANodeId}";
                if (!usedEdges.Add(edgeKey))
                    continue;

                var go = new GameObject($"{edge.ANodeId}_{edge.BNodeId}");
                go.transform.SetParent(root.transform, false);
                go.AddComponent<MapfEdge>().Configure(a, b);
            }
        }

        private Dictionary<string, PalletMarker> SpawnPallets(
            TaskPlanningScenario scenario,
            IReadOnlyDictionary<string, MapfNode> nodes)
        {
            var root = CreateChildRoot("Pallets");
            var pallets = new Dictionary<string, PalletMarker>();
            foreach (var palletDefinition in scenario.Pallets)
            {
                if (!nodes.TryGetValue(palletDefinition.CurrentNodeId, out var currentNode))
                {
                    Debug.LogWarning($"Skipping pallet '{palletDefinition.PalletId}' because current node '{palletDefinition.CurrentNodeId}' is missing.", this);
                    continue;
                }

                nodes.TryGetValue(palletDefinition.ParkingNodeId, out var parkingNode);

                var go = new GameObject(palletDefinition.PalletId);
                go.transform.SetParent(root.transform, false);
                go.transform.position = new Vector3(currentNode.transform.position.x, currentNode.transform.position.y, -0.05f);

                var renderer = go.AddComponent<SpriteRenderer>();
                renderer.sprite = palletSprite;
                renderer.color = palletColor;

                var pallet = go.AddComponent<PalletMarker>();
                pallet.Configure(
                    palletDefinition.PalletId,
                    currentNode,
                    parkingNode,
                    palletDefinition.AttachSeconds,
                    palletDefinition.DetachSeconds,
                    palletDefinition.LoadSeconds,
                    palletDefinition.UnloadSeconds);
                pallets[palletDefinition.PalletId] = pallet;
            }

            return pallets;
        }

        private List<PalletLoadingPoint> SpawnLoadingPoints(
            TaskPlanningScenario scenario,
            IReadOnlyDictionary<string, MapfNode> nodes,
            IReadOnlyDictionary<string, PalletMarker> pallets)
        {
            var root = CreateChildRoot("Loading Points");
            var loadingPoints = new List<PalletLoadingPoint>();
            foreach (var definition in scenario.LoadingPoints)
            {
                if (!nodes.TryGetValue(definition.NodeId, out var node))
                {
                    Debug.LogWarning($"Skipping loading point '{definition.LoadingPointId}' because node '{definition.NodeId}' is missing.", this);
                    continue;
                }

                var go = new GameObject(definition.LoadingPointId);
                go.transform.SetParent(root.transform, false);
                var point = go.AddComponent<PalletLoadingPoint>();
                point.Configure(definition.LoadingPointId, node, ResolvePallets(definition.AcceptedPalletIds, pallets));
                loadingPoints.Add(point);
            }

            return loadingPoints;
        }

        private void SpawnWorkstations(
            TaskPlanningScenario scenario,
            IReadOnlyDictionary<string, MapfNode> nodes,
            IReadOnlyDictionary<string, PalletMarker> pallets)
        {
            var root = CreateChildRoot("Workstations");
            foreach (var definition in scenario.Workstations)
            {
                if (!nodes.TryGetValue(definition.NodeId, out var node))
                {
                    Debug.LogWarning($"Skipping workstation '{definition.WorkstationId}' because node '{definition.NodeId}' is missing.", this);
                    continue;
                }

                var go = new GameObject(definition.WorkstationId);
                go.transform.SetParent(root.transform, false);
                go.AddComponent<WorkstationDeliveryPoint>()
                    .Configure(definition.WorkstationId, node, ResolvePallets(definition.AcceptedPalletIds, pallets));
            }
        }

        private List<TaskPlanningAmr> SpawnAmrs(
            TaskPlanningScenario scenario,
            IReadOnlyDictionary<string, MapfNode> nodes)
        {
            var root = CreateChildRoot("AMRs");
            var amrs = new List<TaskPlanningAmr>();
            foreach (var definition in scenario.Amrs)
            {
                if (!nodes.TryGetValue(definition.StartNodeId, out var startNode))
                {
                    Debug.LogWarning($"Skipping AMR '{definition.AmrId}' because start node '{definition.StartNodeId}' is missing.", this);
                    continue;
                }

                var go = new GameObject(definition.AmrId);
                go.transform.SetParent(root.transform, false);
                go.transform.position = new Vector3(startNode.transform.position.x, startNode.transform.position.y, -0.1f);

                var renderer = go.AddComponent<SpriteRenderer>();
                renderer.sprite = agentSprite;
                renderer.color = amrColors.Length == 0 ? Color.white : amrColors[definition.AgentId % amrColors.Length];

                go.AddComponent<MapfAgentController>();
                go.AddComponent<MapfAgent>().Configure(definition.AgentId, startNode, startNode);
                var mount = new GameObject("Pallet Mount");
                mount.transform.SetParent(go.transform, false);
                mount.transform.localPosition = Vector3.zero;

                var amr = go.AddComponent<TaskPlanningAmr>();
                amr.Configure(definition.AmrId, mount.transform);
                amrs.Add(amr);
            }

            return amrs;
        }

        private void ConfigureSystemComponents(
            TaskPlanningScenario scenario,
            IReadOnlyList<TaskPlanningAmr> amrs,
            IReadOnlyList<PalletLoadingPoint> loadingPoints)
        {
            var mapfRoot = CreateChildRoot(MapfRootName);
            var sceneGraph = mapfRoot.AddComponent<MapfSceneGraph>();
            var coordinator = mapfRoot.AddComponent<MapfCoordinator>();
            mapfRoot.AddComponent<MapfDebugGizmos>();

            var schedulerRoot = CreateChildRoot(SchedulerRootName);
            var scheduler = schedulerRoot.AddComponent<TaskScheduler>();
            schedulerRoot.AddComponent<TaskPlanningDebugGizmos>();

            var mesRoot = CreateChildRoot(MesRootName);
            var mes = mesRoot.AddComponent<TaskPlanningMes>();
            var metrics = mesRoot.AddComponent<TaskPlanningMetricsCollector>();

            coordinator.Configure(sceneGraph, shouldPlanOnStart: false, shouldLogSceneSnapshotOnStart: true);
            scheduler.ConfigureScene(coordinator, sceneGraph, amrs, loadingPoints);
            mes.ConfigureScheduledScenario(scheduler, CreateMesScheduledScenarioAsset(scenario), autoStartOnPlay: true, batchSameTimestamp: true);
            metrics.Configure(mes, scheduler);
        }

        private TaskPlanningMesScheduledScenarioAsset CreateMesScheduledScenarioAsset(TaskPlanningScenario scenario)
        {
#if UNITY_EDITOR
            if (saveMesScheduledScenarioAssetInProject)
            {
                EnsureScenarioFolders();
                var path = $"{MesScheduledScenarioAssetFolder}/{SanitizeFileName(scenario.Name)}.asset";
                var asset = AssetDatabase.LoadAssetAtPath<TaskPlanningMesScheduledScenarioAsset>(path);
                if (asset == null)
                {
                    asset = ScriptableObject.CreateInstance<TaskPlanningMesScheduledScenarioAsset>();
                    AssetDatabase.CreateAsset(asset, path);
                }

                asset.SetTasks(scenario.ScheduledTasks);
                EditorUtility.SetDirty(asset);
                AssetDatabase.SaveAssets();
                return asset;
            }
#endif

            var transient = TaskPlanningMesScheduledScenarioAsset.Create(scenario.ScheduledTasks);
            transient.name = scenario.Name;
            return transient;
        }

        private GameObject CreateChildRoot(string rootName)
        {
            var root = new GameObject(rootName);
            root.transform.SetParent(transform, false);
            return root;
        }

        private static IReadOnlyList<PalletMarker> ResolvePallets(
            IEnumerable<string> palletIds,
            IReadOnlyDictionary<string, PalletMarker> pallets)
        {
            return palletIds == null
                ? System.Array.Empty<PalletMarker>()
                : palletIds
                    .Where(id => !string.IsNullOrWhiteSpace(id))
                    .Select(id => pallets.TryGetValue(id, out var pallet) ? pallet : null)
                    .Where(pallet => pallet != null)
                    .ToArray();
        }

        private void ClearChildren()
        {
            for (var i = transform.childCount - 1; i >= 0; i--)
            {
                var child = transform.GetChild(i).gameObject;
                if (Application.isPlaying)
                    Destroy(child);
                else
                    DestroyImmediate(child);
            }
        }

        private void RemoveLegacySystemComponentsFromSpawner()
        {
            RemoveLegacySystemComponent<MapfSceneGraph>();
            RemoveLegacySystemComponent<MapfCoordinator>();
            RemoveLegacySystemComponent<MapfDebugGizmos>();
            RemoveLegacySystemComponent<TaskScheduler>();
            RemoveLegacySystemComponent<TaskPlanningDebugGizmos>();
            RemoveLegacySystemComponent<TaskPlanningMes>();
            RemoveLegacySystemComponent<TaskPlanningMetricsCollector>();
        }

        private void RemoveLegacySystemComponent<TComponent>()
            where TComponent : Component
        {
            foreach (var component in GetComponents<TComponent>())
            {
                if (Application.isPlaying)
                    Destroy(component);
                else
                    DestroyImmediate(component);
            }
        }

#if UNITY_EDITOR
        private static void EnsureScenarioFolders()
        {
            EnsureFolder("Assets/TaskPlanning", "Scenarios");
            EnsureFolder(MesScheduledScenarioAssetFolder, "Metrics");
        }

        private static void EnsureFolder(string parent, string child)
        {
            if (!AssetDatabase.IsValidFolder($"{parent}/{child}"))
                AssetDatabase.CreateFolder(parent, child);
        }
#endif

        private static string SanitizeFileName(string value)
        {
            var invalidChars = Path.GetInvalidFileNameChars();
            var chars = value.Select(c => invalidChars.Contains(c) ? '_' : c).ToArray();
            return new string(chars).Replace(' ', '_');
        }
    }
}
