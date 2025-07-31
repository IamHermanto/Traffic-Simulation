using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

public class PropsSorter : EditorWindow
{
    [MenuItem("Tools/City Organizer/Sort Props")]
    public static void ShowWindow()
    {
        GetWindow<PropsSorter>("Props Sorter");
    }

    private Vector2 scrollPosition;

    private void OnGUI()
    {
        GUILayout.Label("Props Organizer with Subcategories", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("This tool will sort props into subcategories based on their first word:", EditorStyles.wordWrappedLabel);
        GUILayout.Label("Props → Air → air conditioner roof, air conditioner wall", EditorStyles.miniLabel);
        GUILayout.Label("Props → Traffic → Traffic light 1, Traffic light 2, traffic_obj", EditorStyles.miniLabel);
        GUILayout.Label("Props → Flower → Flower 1 prefab A, Flower 1 prefab B, etc.", EditorStyles.miniLabel);
        GUILayout.Space(10);
        
        if (GUILayout.Button("Sort All Props Prefabs", GUILayout.Height(30)))
        {
            SortPropsPrefabs();
        }
        
        GUILayout.Space(10);
        
        if (GUILayout.Button("Preview Props Categories", GUILayout.Height(25)))
        {
            PreviewPropsCategories();
        }
        
        GUILayout.Space(5);
        
        if (GUILayout.Button("Clear Props Organization", GUILayout.Height(25)))
        {
            ClearPropsOrganization();
        }
    }
    
    private void SortPropsPrefabs()
    {
        // Create main Props parent
        GameObject propsParent = CreateOrFindParent("Props");
        
        // Get all GameObjects in the scene
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        
        // Dictionary to store props by category
        Dictionary<string, List<GameObject>> propsByCategory = new Dictionary<string, List<GameObject>>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsPropPrefab(obj.name))
            {
                string category = GetPropCategory(obj.name);
                
                if (!propsByCategory.ContainsKey(category))
                {
                    propsByCategory[category] = new List<GameObject>();
                }
                propsByCategory[category].Add(obj);
            }
        }
        
        // Create subcategories and organize props
        int totalSorted = 0;
        foreach (var kvp in propsByCategory.OrderBy(x => x.Key))
        {
            string categoryName = kvp.Key;
            List<GameObject> categoryProps = kvp.Value;
            
            // Create subcategory parent
            GameObject categoryParent = CreateOrFindParent(categoryName, propsParent);
            
            // Move props to their category
            int categoryCount = 0;
            foreach (GameObject prop in categoryProps)
            {
                if (prop.transform.parent != categoryParent.transform)
                {
                    Undo.SetTransformParent(prop.transform, categoryParent.transform, "Sort Props Prefabs");
                    categoryCount++;
                    totalSorted++;
                }
            }
            
            Debug.Log($"Category '{categoryName}': {categoryProps.Count} objects");
        }
        
        Debug.Log("=== PROPS SORTING COMPLETE ===");
        Debug.Log($"Total categories created: {propsByCategory.Count}");
        Debug.Log($"Total props sorted: {totalSorted}");
        
        // Mark scene as dirty
        EditorUtility.SetDirty(propsParent);
    }
    
    private void PreviewPropsCategories()
    {
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        Dictionary<string, List<string>> propsByCategory = new Dictionary<string, List<string>>();
        
        foreach (GameObject obj in allObjects)
        {
            if (IsPropPrefab(obj.name))
            {
                string category = GetPropCategory(obj.name);
                
                if (!propsByCategory.ContainsKey(category))
                {
                    propsByCategory[category] = new List<string>();
                }
                propsByCategory[category].Add(obj.name);
            }
        }
        
        Debug.Log("=== PROPS CATEGORIES PREVIEW ===");
        
        foreach (var kvp in propsByCategory.OrderBy(x => x.Key))
        {
            Debug.Log($"--- CATEGORY: {kvp.Key.ToUpper()} ({kvp.Value.Count} items) ---");
            foreach (string propName in kvp.Value.OrderBy(x => x))
            {
                Debug.Log($"  • {propName}");
            }
        }
        
        Debug.Log($"Total categories: {propsByCategory.Count}");
        Debug.Log($"Total props: {propsByCategory.Values.Sum(list => list.Count)}");
    }
    
    private void ClearPropsOrganization()
    {
        GameObject propsParent = GameObject.Find("Props");
        if (propsParent != null)
        {
            if (EditorUtility.DisplayDialog("Clear Props Organization", 
                "This will move all props back to the root and delete the Props organization structure. Are you sure?", 
                "Yes, Clear", "Cancel"))
            {
                // Move all prop objects back to root
                Transform[] allChildren = propsParent.GetComponentsInChildren<Transform>();
                foreach (Transform child in allChildren)
                {
                    if (child != propsParent.transform && IsPropPrefab(child.name))
                    {
                        Undo.SetTransformParent(child, null, "Clear Props Organization");
                    }
                }
                
                // Delete the Props parent object
                Undo.DestroyObjectImmediate(propsParent);
                Debug.Log("Props organization cleared successfully.");
            }
        }
        else
        {
            Debug.Log("No Props organization found to clear.");
        }
    }
    
    private GameObject CreateOrFindParent(string parentName, GameObject parent = null)
    {
        GameObject existingParent = null;
        
        if (parent == null)
        {
            existingParent = GameObject.Find(parentName);
        }
        else
        {
            Transform childTransform = parent.transform.Find(parentName);
            if (childTransform != null)
                existingParent = childTransform.gameObject;
        }
        
        if (existingParent == null)
        {
            existingParent = new GameObject(parentName);
            if (parent != null)
            {
                existingParent.transform.SetParent(parent.transform);
            }
            Undo.RegisterCreatedObjectUndo(existingParent, $"Create {parentName} Parent");
        }
        
        return existingParent;
    }
    
    private bool IsPropPrefab(string objectName)
    {
        string name = objectName.ToLower();
        
        // Define non-prop categories to exclude
        string[] excludeCategories = {
            "building", "build_", "lamp_", "street lamp", "walkway", "street", "sideway", "stoneway"
        };
        
        foreach (string exclude in excludeCategories)
        {
            if (name.StartsWith(exclude))
            {
                return false;
            }
        }
        
        // If it contains "prefab" and isn't excluded, it's likely a prop
        if (name.Contains("prefab"))
        {
            return true;
        }
        
        // Additional patterns for props that might not have "prefab" in the name
        string[] propKeywords = {
            "sign", "pot", "trash", "tree", "wall", "fence", "flower", "grass", "bush", 
            "bench", "hydrant", "drain", "pole", "box", "can_", "bin", "atm", "phone booth",
            "air conditioner", "water tank", "wrench", "ketchup", "burger", "hotdog", "mayonnaise"
        };
        
        foreach (string keyword in propKeywords)
        {
            if (name.Contains(keyword))
            {
                return true;
            }
        }
        
        return false;
    }
    
    private string GetPropCategory(string objectName)
    {
        string name = objectName.Trim();
        
        // Handle special cases for multi-word categories
        if (name.ToLower().StartsWith("air conditioner"))
            return "Air";
        if (name.ToLower().StartsWith("phone booth"))
            return "Phone";
        if (name.ToLower().StartsWith("bus stop"))
            return "Bus";
        if (name.ToLower().StartsWith("big_trash") || name.ToLower().StartsWith("big trash"))
            return "Trash";
        if (name.ToLower().StartsWith("water tank"))
            return "Water";
        if (name.ToLower().StartsWith("wood pot"))
            return "Pot";
        if (name.ToLower().StartsWith("stone pot"))
            return "Pot";
        if (name.ToLower().StartsWith("pot_tree"))
            return "Pot";
        if (name.ToLower().StartsWith("shopping cart"))
            return "Shopping";
        if (name.ToLower().StartsWith("parking tower") || name.ToLower().StartsWith("parking_barrier"))
            return "Parking";
        if (name.ToLower().StartsWith("mail_box"))
            return "Mail";
        if (name.ToLower().StartsWith("electricty box"))
            return "Electric";
        if (name.ToLower().StartsWith("dynamic_power"))
            return "Power";
        if (name.ToLower().StartsWith("power_poles"))
            return "Power";
        if (name.ToLower().StartsWith("traffic"))
            return "Traffic";
        if (name.ToLower().StartsWith("trashbag") || name.ToLower().StartsWith("trashcan"))
            return "Trash";
        if (name.ToLower().StartsWith("streetprop") || name.ToLower().StartsWith("streetsellerstand"))
            return "Street";
        if (name.ToLower().StartsWith("televsrion"))
            return "Antenna";
        if (name.ToLower().StartsWith("hedge"))
            return "Hedge";
        if (name.ToLower().StartsWith("colamachine") || name.ToLower().StartsWith("glasscola"))
            return "Drink";
        if (name.ToLower().StartsWith("can_"))
            return "Can";
        if (name.ToLower().StartsWith("fence"))
            return "Fence";
        if (name.ToLower().StartsWith("flower"))
            return "Flower";
        if (name.ToLower().StartsWith("grass"))
            return "Grass";
        if (name.ToLower().StartsWith("stone"))
            return "Stone";
        if (name.ToLower().StartsWith("platt"))
            return "Platt";
        if (name.ToLower().StartsWith("hospital_sign"))
            return "Hospital";
        if (name.ToLower().StartsWith("burgersh"))
            return "Burger";
        if (name.ToLower().StartsWith("bank_sign"))
            return "Bank";
        
        // Default: take the first word before space or underscore
        string[] separators = { " ", "_" };
        string firstWord = name.Split(separators, System.StringSplitOptions.RemoveEmptyEntries)[0];
        
        // Capitalize first letter for consistency
        if (!string.IsNullOrEmpty(firstWord))
        {
            return char.ToUpper(firstWord[0]) + firstWord.Substring(1).ToLower();
        }
        
        return "Misc";
    }
}

// Quick menu option for instant sorting
public class AutoPropsSorter
{
    [MenuItem("Tools/City Organizer/Auto Sort All Props")]
    public static void AutoSortProps()
    {
        GameObject propsParent = CreateOrFindParent("Props");
        GameObject[] allObjects = Object.FindObjectsOfType<GameObject>();
        
        Dictionary<string, GameObject> categoryParents = new Dictionary<string, GameObject>();
        int sortedCount = 0;
        
        foreach (GameObject obj in allObjects)
        {
            if (IsPropObject(obj.name) && obj.transform.parent == null)
            {
                string category = GetCategory(obj.name);
                
                if (!categoryParents.ContainsKey(category))
                {
                    categoryParents[category] = CreateOrFindParent(category, propsParent);
                }
                
                Undo.SetTransformParent(obj.transform, categoryParents[category].transform, "Auto Sort Props");
                sortedCount++;
            }
        }
        
        Debug.Log($"Auto Props Sorter: Organized {sortedCount} props into {categoryParents.Count} categories");
        EditorUtility.SetDirty(propsParent);
    }
    
    private static GameObject CreateOrFindParent(string parentName, GameObject parent = null)
    {
        GameObject existingParent = null;
        
        if (parent == null)
        {
            existingParent = GameObject.Find(parentName);
        }
        else
        {
            Transform childTransform = parent.transform.Find(parentName);
            if (childTransform != null)
                existingParent = childTransform.gameObject;
        }
        
        if (existingParent == null)
        {
            existingParent = new GameObject(parentName);
            if (parent != null)
            {
                existingParent.transform.SetParent(parent.transform);
            }
            Undo.RegisterCreatedObjectUndo(existingParent, $"Create {parentName} Parent");
        }
        
        return existingParent;
    }
    
    private static bool IsPropObject(string objectName)
    {
        string name = objectName.ToLower();
        
        // Exclude already categorized items
        if (name.StartsWith("building") || name.StartsWith("build_") || 
            name.StartsWith("lamp_") || name.StartsWith("street lamp") ||
            name.StartsWith("walkway") || name.StartsWith("street ") || 
            name.StartsWith("sideway") || name.StartsWith("stoneway"))
        {
            return false;
        }
        
        return name.Contains("prefab") || 
               name.Contains("sign") || name.Contains("pot") || name.Contains("trash") ||
               name.Contains("tree") || name.Contains("wall") || name.Contains("fence") ||
               name.Contains("flower") || name.Contains("grass") || name.Contains("bush");
    }
    
    private static string GetCategory(string objectName)
    {
        // Simplified version for quick sorting
        string name = objectName.ToLower();
        
        if (name.StartsWith("air")) return "Air";
        if (name.StartsWith("traffic")) return "Traffic";
        if (name.StartsWith("flower")) return "Flower";
        if (name.StartsWith("grass")) return "Grass";
        if (name.StartsWith("stone")) return "Stone";
        if (name.StartsWith("fence")) return "Fence";
        if (name.StartsWith("can_")) return "Can";
        if (name.StartsWith("bush")) return "Bush";
        if (name.StartsWith("wall")) return "Wall";
        if (name.StartsWith("tree")) return "Tree";
        if (name.StartsWith("trash")) return "Trash";
        if (name.StartsWith("pot") || name.Contains("pot")) return "Pot";
        
        string firstWord = name.Split(' ', '_')[0];
        return char.ToUpper(firstWord[0]) + firstWord.Substring(1);
    }
}