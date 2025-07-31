#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(TrafficSystemAnalyzer))]
public class TrafficSystemAnalyzerEditor : Editor
{
    private bool showAdvancedSettings = false;
    private bool showAnalysisResults = false;
    private bool showStreetDetails = false;
    
    public override void OnInspectorGUI()
    {
        TrafficSystemAnalyzer analyzer = (TrafficSystemAnalyzer)target;
        
        // Header
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("3D Traffic System - Street Analyzer", EditorStyles.largeLabel);
        EditorGUILayout.LabelField("Step 1: Analyze street network and connections", EditorStyles.miniLabel);
        EditorGUILayout.Space();
        
        // Main Analysis Button
        EditorGUILayout.BeginVertical("box");
        EditorGUILayout.LabelField("Street Network Analysis", EditorStyles.boldLabel);
        
        if (GUILayout.Button("🔍 Analyze Street Network", GUILayout.Height(40)))
        {
            analyzer.AnalyzeStreetNetwork();
            SceneView.RepaintAll();
        }
        
        EditorGUILayout.Space();
        
        // Quick stats
        if (analyzer.HasAnalyzedStreets)
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField($"Streets: {analyzer.DetectedStreets.Count}", GUILayout.Width(80));
            EditorGUILayout.LabelField($"Intersections: {analyzer.Intersections.Count}", GUILayout.Width(120));
            EditorGUILayout.EndHorizontal();
        }
        
        EditorGUILayout.EndVertical();
        
        // Settings
        EditorGUILayout.Space();
        showAdvancedSettings = EditorGUILayout.Foldout(showAdvancedSettings, "⚙️ Analysis Settings", true);
        if (showAdvancedSettings)
        {
            EditorGUILayout.BeginVertical("box");
            DrawDefaultInspector();
            EditorGUILayout.EndVertical();
        }
        
        // Analysis Results
        if (analyzer.HasAnalyzedStreets)
        {
            EditorGUILayout.Space();
            showAnalysisResults = EditorGUILayout.Foldout(showAnalysisResults, "📊 Analysis Results", true);
            if (showAnalysisResults)
            {
                DrawAnalysisResults(analyzer);
            }
            
            // Street Details
            EditorGUILayout.Space();
            showStreetDetails = EditorGUILayout.Foldout(showStreetDetails, "🛣️ Street Details", true);
            if (showStreetDetails)
            {
                DrawStreetDetails(analyzer);
            }
        }
        
        // Help and Next Steps
        EditorGUILayout.Space();
        DrawHelpSection(analyzer);
    }
    
    private void DrawAnalysisResults(TrafficSystemAnalyzer analyzer)
    {
        EditorGUILayout.BeginVertical("box");
        
        // Summary stats
        EditorGUILayout.LabelField("Network Statistics:", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField($"Total Streets: {analyzer.DetectedStreets.Count}");
        EditorGUILayout.LabelField($"Intersections: {analyzer.Intersections.Count}");
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.Space();
        
        // Intersection details
        if (analyzer.Intersections.Count > 0)
        {
            EditorGUILayout.LabelField("Intersection Points:", EditorStyles.boldLabel);
            foreach (StreetSegment intersection in analyzer.Intersections)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField($"• {intersection.streetObject.name}", GUILayout.Width(150));
                EditorGUILayout.LabelField($"({intersection.connectionCount} connections)", EditorStyles.miniLabel);
                
                if (GUILayout.Button("Select", GUILayout.Width(60)))
                {
                    Selection.activeGameObject = intersection.streetObject;
                    SceneView.FrameLastActiveSceneView();
                }
                EditorGUILayout.EndHorizontal();
            }
        }
        
        // Dead ends
        var deadEnds = analyzer.GetDeadEndStreets();
        if (deadEnds.Count > 0)
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField($"Dead Ends ({deadEnds.Count}):", EditorStyles.boldLabel);
            foreach (StreetSegment deadEnd in deadEnds)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField($"• {deadEnd.streetObject.name}", GUILayout.Width(150));
                if (GUILayout.Button("Select", GUILayout.Width(60)))
                {
                    Selection.activeGameObject = deadEnd.streetObject;
                    SceneView.FrameLastActiveSceneView();
                }
                EditorGUILayout.EndHorizontal();
            }
        }
        
        EditorGUILayout.EndVertical();
    }
    
    private void DrawStreetDetails(TrafficSystemAnalyzer analyzer)
    {
        EditorGUILayout.BeginVertical("box");
        
        if (analyzer.DetectedStreets.Count > 10)
        {
            EditorGUILayout.LabelField($"Showing first 10 of {analyzer.DetectedStreets.Count} streets:");
        }
        
        int maxDisplay = Mathf.Min(analyzer.DetectedStreets.Count, 10);
        
        for (int i = 0; i < maxDisplay; i++)
        {
            StreetSegment street = analyzer.DetectedStreets[i];
            EditorGUILayout.BeginHorizontal("box");
            
            // Street name and type
            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField(street.streetObject.name, EditorStyles.boldLabel);
            
            string streetInfo = $"Connections: {street.connectionCount}";
            if (street.isIntersection)
                streetInfo += " (Intersection)";
            
            EditorGUILayout.LabelField(streetInfo, EditorStyles.miniLabel);
            
            if (street.connectionCount > 0)
            {
                EditorGUILayout.LabelField($"Connected to: {street.GetConnectionNames()}", EditorStyles.miniLabel);
            }
            EditorGUILayout.EndVertical();
            
            // Select button
            EditorGUILayout.BeginVertical(GUILayout.Width(60));
            if (GUILayout.Button("Select"))
            {
                Selection.activeGameObject = street.streetObject;
                SceneView.FrameLastActiveSceneView();
            }
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.EndHorizontal();
        }
        
        if (analyzer.DetectedStreets.Count > 10)
        {
            EditorGUILayout.LabelField($"... and {analyzer.DetectedStreets.Count - 10} more streets", EditorStyles.miniLabel);
        }
        
        EditorGUILayout.EndVertical();
    }
    
    private void DrawHelpSection(TrafficSystemAnalyzer analyzer)
    {
        EditorGUILayout.BeginVertical("box");
        EditorGUILayout.LabelField("Instructions:", EditorStyles.boldLabel);
        
        if (!analyzer.HasAnalyzedStreets)
        {
            EditorGUILayout.HelpBox(
                "1. Click 'Analyze Street Network' to detect all streets\n" +
                "2. Check Scene view for visual debug info (blue lines = streets, green = connections, red = intersections)\n" +
                "3. Check Console for detailed analysis logs", 
                MessageType.Info);
        }
        else
        {
            EditorGUILayout.HelpBox(
                "✅ Street analysis complete!\n\n" +
                "Next Steps:\n" +
                "• Step 2: Generate waypoints along streets\n" +
                "• Step 3: Create traffic lanes\n" +
                "• Step 4: Add traffic simulation logic", 
                MessageType.Info);
            
            if (GUILayout.Button("Clear Analysis & Re-analyze"))
            {
                analyzer.AnalyzeStreetNetwork();
                SceneView.RepaintAll();
            }
        }
        
        EditorGUILayout.Space();
        
        // Debug visualization controls
        EditorGUILayout.LabelField("Scene Visualization:", EditorStyles.boldLabel);
        bool newShowGizmos = EditorGUILayout.Toggle("Show Debug Gizmos", analyzer.showDebugGizmos);
        if (newShowGizmos != analyzer.showDebugGizmos)
        {
            analyzer.showDebugGizmos = newShowGizmos;
            SceneView.RepaintAll();
        }
        
        EditorGUILayout.EndVertical();
    }
}
#endif