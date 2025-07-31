using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

public class BuildingSorter : EditorWindow
{
    [MenuItem("Tools/City Organizer/Sort Buildings")]
    public static void ShowWindow()
    {
        GetWindow<BuildingSorter>("Building Sorter");
    }

    private void OnGUI()
    {
        GUILayout.Label("City 3D Asset Organizer", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool will automatically sort building prefabs in your scene.", EditorStyles.wordWrappedLabel);
        GUILayout.Space(10);
        
        if (GUILayout.Button("Sort Building Prefabs", GUILayout.Height(30)))
        {
            SortBuildingPrefabs();
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Preview Building Objects", GUILayout.Height(25)))
        {
            PreviewBuildingObjects();
        }
    }
    
    private void SortBuildingPrefabs()
    {
        // Find or create the Buildings parent object
        GameObject buildingsParent = GameObject.Find("Buildings");
        if (buildingsParent == null)
        {
            buildingsParent = new GameObject("Buildings");
            Undo.RegisterCreatedObjectUndo(buildingsParent, "Create Buildings Parent");
        }
        
        // Get all GameObjects in the scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<GameObject> buildingObjects = new List<GameObject>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsBuildingPrefab(obj.name))
            {
                buildingObjects.Add(obj);
            }
        }
        
        // Move building objects under the Buildings parent
        int movedCount = 0;
        foreach (GameObject building in buildingObjects)
        {
            // Only move if it's not already a child of Buildings parent
            if (building.transform.parent != buildingsParent.transform)
            {
                Undo.SetTransformParent(building.transform, buildingsParent.transform, "Sort Building Prefabs");
                movedCount++;
            }
        }
        
        Debug.Log($"Building Sorter: Moved {movedCount} building prefabs under 'Buildings' parent. Total building objects: {buildingObjects.Count}");
        
        // Mark scene as dirty to save changes
        EditorUtility.SetDirty(buildingsParent);
    }
    
    private void PreviewBuildingObjects()
    {
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        List<string> buildingNames = new List<string>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsBuildingPrefab(obj.name))
            {
                buildingNames.Add(obj.name);
            }
        }
        
        Debug.Log("=== BUILDING PREFABS FOUND ===");
        foreach (string name in buildingNames)
        {
            Debug.Log($"Building: {name}");
        }
        Debug.Log($"Total building prefabs found: {buildingNames.Count}");
    }
    
    private bool IsBuildingPrefab(string objectName)
    {
        // Convert to lowercase for case-insensitive comparison
        string name = objectName.ToLower();
        
        // Primary pattern: starts with "building_"
        if (name.StartsWith("building_"))
        {
            return true;
        }
        
        // Secondary pattern: starts with "build_"
        if (name.StartsWith("build_"))
        {
            return true;
        }
        
        // Specific building-related prefabs
        string[] specificBuildingPrefabs = {
            "bank_prefab",
            "hospital_prefab",
            "motel_prefab",
            "police_station_prefab",
            "fire_department_prefab",
            "supermarket_prefab"
        };
        
        foreach (string buildingType in specificBuildingPrefabs)
        {
            if (name == buildingType)
            {
                return true;
            }
        }
        
        return false;
    }
}

// Alternative: Automatic sorting script that can be run from the menu
public class AutoBuildingSorter
{
    [MenuItem("Tools/City Organizer/Auto Sort All Buildings")]
    public static void AutoSortBuildings()
    {
        // Create parent objects
        GameObject buildingsParent = CreateOrFindParent("Buildings");
        
        // Get all objects and sort them
        GameObject[] allObjects = Object.FindObjectsOfType<GameObject>();
        int sortedCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            if (IsBuildingObject(obj.name) && obj.transform.parent == null)
            {
                Undo.SetTransformParent(obj.transform, buildingsParent.transform, "Auto Sort Buildings");
                sortedCount++;
            }
        }
        
        Debug.Log($"Auto Building Sorter: Organized {sortedCount} building objects");
        EditorUtility.SetDirty(buildingsParent);
    }
    
    private static GameObject CreateOrFindParent(string parentName)
    {
        GameObject parent = GameObject.Find(parentName);
        if (parent == null)
        {
            parent = new GameObject(parentName);
            Undo.RegisterCreatedObjectUndo(parent, $"Create {parentName} Parent");
        }
        return parent;
    }
    
    private static bool IsBuildingObject(string objectName)
    {
        string name = objectName.ToLower();
        
        return name.StartsWith("building_") || 
               name.StartsWith("build_") ||
               name == "bank_prefab" ||
               name == "hospital_prefab" ||
               name == "motel_prefab" ||
               name == "police_station_prefab" ||
               name == "fire_department_prefab" ||
               name == "supermarket_prefab";
    }
}