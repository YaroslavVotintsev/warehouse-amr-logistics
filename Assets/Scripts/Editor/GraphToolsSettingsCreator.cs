#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

public static class GraphToolsSettingsCreator
{
    [MenuItem("Tools/Graph/Create Settings Asset")]
    public static void CreateSettings()
    {
        var asset = ScriptableObject.CreateInstance<GraphToolsSettings>();
        AssetDatabase.CreateAsset(asset, "Assets/GraphToolsSettings.asset");
        AssetDatabase.SaveAssets();

        Selection.activeObject = asset;
    }
}
#endif