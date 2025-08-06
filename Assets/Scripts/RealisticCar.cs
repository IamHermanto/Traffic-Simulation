using System.Collections.Generic;
using UnityEngine;

public class RealisticCar : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel; 
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;
    
    [Header("Wheel Meshes")]
    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;
    
    [Header("Car Settings")]
    public float maxMotorTorque = 1500f;
    public float maxSteerAngle = 45f;
    public float maxBrakeTorque = 3000f;
    
    [Header("🔍 BEHAVIOR LOGGING")]
    [Tooltip("Enable comprehensive behavior logging")]
    public bool enableBehaviorLogging = true;
    [Tooltip("Log steering decisions and reasons")]
    public bool logSteeringDecisions = true;
    [Tooltip("Log speed decisions and reasons")]
    public bool logSpeedDecisions = true;
    [Tooltip("Log lane change safety checks")]
    public bool logLaneChanges = true;
    [Tooltip("Log intersection behavior")]
    public bool logIntersections = true;
    [Tooltip("Log path following decisions")]
    public bool logPathFollowing = true;
    [Tooltip("How often to log (seconds)")]
    public float loggingInterval = 1f;
    
    [Header("Turning Settings")]
    public bool aggressiveTurning = true;
    public float maxSteerAnglePerFrame = 8f;
    public float turnSlowdownFactor = 0.7f;
    public float turnDetectionAngle = 25f;
    
    [Header("Dynamic Speed Control")]
    public float straightRoadSpeedMultiplier = 2.5f;
    public float straightRoadMinDistance = 8f;
    public bool enableSpeedBoost = true;
    
    [Header("AI Pathfinding")]
    public List<Node> path = new List<Node>();
    public int currentNodeIndex = 0;
    public float stoppingDistance = 1.5f;
    public float detectionRange = 5f;
    public float nodeReachDistance = 1.8f;
    public float despawnDelay = 1f;
    public float maxLookAheadDistance = 4f;
    public float pathRecalculateInterval = 2f;
    
    [Header("Lane Change Safety")]
    public bool enableLaneChangeSafety = true;
    public float laneWidth = 3f;
    public float laneChangeSafetyDistance = 8f;
    public float sideScanDistance = 10f;
    
    [Header("Road Constraint")]
    public bool strictRoadFollowing = true;
    public float roadWidthTolerance = 3f;
    
    [Header("Intersection Behavior")]
    public float intersectionDetectionRange = 4f;
    public float intersectionSlowdownFactor = 0.6f;
    public float intersectionStopDistance = 1.2f;
    public float otherCarDetectionRange = 3f;
    
    [Header("Performance Settings")]
    public float obstacleCheckInterval = 0.1f;
    
    // Core variables
    Rigidbody rb;
    bool isMoving = false;
    bool isBlocked = false;
    bool isAtIntersection = false;
    bool shouldYieldAtIntersection = false;
    bool isOffRoad = false;
    
    private float obstacleCheckTimer;
    private float pathRecalculateTimer;
    private Node targetEndNode;
    private bool hasInitialAlignment = false;
    private float alignmentTime = 0f;
    private float lastSteerAngle = 0f;
    
    // Intersection detection cache
    private List<Node> nearbyIntersectionNodes = new List<Node>();
    private float intersectionCheckTimer = 0f;
    private float intersectionCheckInterval = 0.2f;
    
    // 🔍 LOGGING VARIABLES
    private float lastLogTime = 0f;
    private string carID;
    private Dictionary<string, object> lastDecisionReasons = new Dictionary<string, object>();
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0.5f);
        
        // Generate unique car ID for logging
        carID = $"Car_{gameObject.GetInstanceID().ToString().Substring(0, 4)}";
        
        ValidateWheelSetup();
        SetupWheelFriction();
        
        if (enableBehaviorLogging)
        {
            LogBehavior($"🚗 {carID} SPAWNED", "Car initialized and ready to drive");
        }
    }
    
    void ValidateWheelSetup()
    {
        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        Transform[] meshes = { frontLeftMesh, frontRightMesh, rearLeftMesh, rearRightMesh };
        string[] wheelNames = { "Front Left", "Front Right", "Rear Left", "Rear Right" };
        
        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] == null)
            {
                Debug.LogError($"Missing {wheelNames[i]} wheel collider on {gameObject.name}");
            }
            if (meshes[i] == null)
            {
                Debug.LogWarning($"Missing {wheelNames[i]} wheel mesh on {gameObject.name}");
            }
        }
    }
    
    void SetupWheelFriction()
    {
        WheelFrictionCurve forwardFriction = new WheelFrictionCurve();
        forwardFriction.extremumSlip = 0.4f;
        forwardFriction.extremumValue = 1.2f;
        forwardFriction.asymptoteSlip = 0.8f;
        forwardFriction.asymptoteValue = 0.7f;
        forwardFriction.stiffness = 1.2f;

        WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve();
        sidewaysFriction.extremumSlip = 0.3f;
        sidewaysFriction.extremumValue = 1.3f;
        sidewaysFriction.asymptoteSlip = 0.5f;
        sidewaysFriction.asymptoteValue = 0.9f;
        sidewaysFriction.stiffness = 1.5f;

        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        
        foreach (WheelCollider wheel in wheels)
        {
            if (wheel == null) continue;
            wheel.forwardFriction = forwardFriction;
            wheel.sidewaysFriction = sidewaysFriction;
        }
    }
    
    void Update()
    {
        if (hasInitialAlignment)
        {
            alignmentTime += Time.deltaTime;
        }
        
        if (isMoving && path.Count > 0)
        {
            obstacleCheckTimer += Time.deltaTime;
            if (obstacleCheckTimer >= obstacleCheckInterval)
            {
                CheckForObstacles();
                CheckForOtherCars();
                CheckRoadConstraints();
                obstacleCheckTimer = 0f;
            }
            
            intersectionCheckTimer += Time.deltaTime;
            if (intersectionCheckTimer >= intersectionCheckInterval)
            {
                DetectIntersections();
                intersectionCheckTimer = 0f;
            }
            
            pathRecalculateTimer += Time.deltaTime;
            if (pathRecalculateTimer >= pathRecalculateInterval && targetEndNode != null)
            {
                RecalculatePath();
                pathRecalculateTimer = 0f;
            }
            
            AIController();
        }
        
        UpdateWheelMeshes();
    }
    
    public void SetPath(List<Node> newPath)
    {
        if (newPath == null || newPath.Count == 0)
        {
            LogBehavior("❌ INVALID PATH", "No path provided");
            return;
        }
        
        path = newPath;
        currentNodeIndex = 0;
        isMoving = true;
        targetEndNode = newPath[newPath.Count - 1];
        
        AlignToRoadDirection();
        
        if (enableBehaviorLogging && logPathFollowing)
        {
            LogBehavior("🎯 PATH SET", $"Following {newPath.Count} nodes to {targetEndNode.name}");
        }
    }
    
    public void SetPathWithoutAlignment(List<Node> newPath)
    {
        if (newPath == null || newPath.Count == 0)
        {
            LogBehavior("❌ INVALID PATH", "No path provided (no alignment)");
            return;
        }
        
        path = newPath;
        currentNodeIndex = 0;
        isMoving = true;
        targetEndNode = newPath[newPath.Count - 1];
        hasInitialAlignment = true;
        alignmentTime = 0f;
        
        if (enableBehaviorLogging && logPathFollowing)
        {
            LogBehavior("🎯 PATH SET (NO ALIGN)", $"Route: {path[0].name} → {targetEndNode.name} ({newPath.Count} nodes)");
        }
    }
    
    void AlignToRoadDirection()
    {
        if (path == null || path.Count < 1) return;
        
        Node startNode = path[0];
        if (startNode == null) return;
        
        Vector3 roadDirection = startNode.GetSpawnDirection();
        
        if (roadDirection.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(roadDirection);
            transform.rotation = targetRotation;
            hasInitialAlignment = true;
            alignmentTime = 0f;
            
            if (enableBehaviorLogging)
            {
                LogBehavior("🧭 ALIGNED", $"Facing {roadDirection} from {startNode.name}");
            }
        }
    }
    
    void CheckRoadConstraints()
    {
        if (!strictRoadFollowing) return;
        
        float distanceToNearestNode = GetDistanceToNearestPathNode();
        bool wasOffRoad = isOffRoad;
        
        if (distanceToNearestNode > roadWidthTolerance)
        {
            isOffRoad = true;
            if (!wasOffRoad && enableBehaviorLogging)
            {
                LogBehavior("🚫 OFF-ROAD", $"Distance to path: {distanceToNearestNode:F1}m (tolerance: {roadWidthTolerance:F1}m)");
            }
        }
        else
        {
            if (wasOffRoad && enableBehaviorLogging)
            {
                LogBehavior("✅ BACK ON ROAD", $"Distance to path: {distanceToNearestNode:F1}m");
            }
            isOffRoad = false;
        }
    }
    
    float GetDistanceToNearestPathNode()
    {
        if (path == null || path.Count == 0) return float.MaxValue;
        
        float nearestDistance = float.MaxValue;
        
        for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count); i++)
        {
            if (path[i] != null)
            {
                float distance = Vector3.Distance(transform.position, path[i].transform.position);
                nearestDistance = Mathf.Min(nearestDistance, distance);
            }
        }
        
        return nearestDistance;
    }
    
    void DetectIntersections()
    {
        nearbyIntersectionNodes.Clear();
        bool wasAtIntersection = isAtIntersection;
        bool wasYielding = shouldYieldAtIntersection;
        isAtIntersection = false;
        
        for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count); i++)
        {
            if (path[i] == null) continue;
            
            float distanceToNode = Vector3.Distance(transform.position, path[i].transform.position);
            
            if (IsIntersectionNode(path[i]) && distanceToNode < intersectionDetectionRange)
            {
                nearbyIntersectionNodes.Add(path[i]);
                
                if (distanceToNode < intersectionStopDistance)
                {
                    isAtIntersection = true;
                }
            }
        }
        
        shouldYieldAtIntersection = isAtIntersection && ShouldYieldAtIntersection();
        
        // Log intersection state changes
        if (enableBehaviorLogging && logIntersections)
        {
            if (!wasAtIntersection && isAtIntersection)
            {
                LogBehavior("🚦 INTERSECTION DETECTED", $"At intersection, should yield: {shouldYieldAtIntersection}");
            }
            if (!wasYielding && shouldYieldAtIntersection)
            {
                LogBehavior("⏸️ YIELDING", "Stopping for right-of-way");
            }
            if (wasYielding && !shouldYieldAtIntersection)
            {
                LogBehavior("▶️ PROCEEDING", "Clear to go through intersection");
            }
        }
    }
    
    bool IsIntersectionNode(Node node)
    {
        List<Node> neighbors = node.GetNeighbors();
        return neighbors.Count > 2;
    }
    
    bool ShouldYieldAtIntersection()
    {
        Collider[] nearbyCars = Physics.OverlapSphere(transform.position, otherCarDetectionRange);
        
        foreach (Collider col in nearbyCars)
        {
            RealisticCar otherCar = col.GetComponent<RealisticCar>();
            if (otherCar != null && otherCar != this)
            {
                Vector3 localRelativePos = transform.InverseTransformPoint(otherCar.transform.position);
                
                if (localRelativePos.x > 0 && otherCar.isMoving)
                {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    void CheckForObstacles()
    {
        RaycastHit hit;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        bool wasBlocked = isBlocked;
        
        if (Physics.Raycast(rayStart, transform.forward, out hit, detectionRange))
        {
            if (hit.collider != null && 
                !hit.collider.CompareTag("Ground") && 
                !hit.collider.CompareTag("Road") &&
                hit.collider.gameObject != gameObject)
            {
                isBlocked = true;
                
                if (!wasBlocked && enableBehaviorLogging)
                {
                    string obstacleType = hit.collider.GetComponent<RealisticCar>() != null ? "CAR" : "OBSTACLE";
                    LogBehavior($"🛑 {obstacleType} DETECTED", $"Blocked by {hit.collider.name} at {hit.distance:F1}m");
                }
                return;
            }
        }
        
        if (wasBlocked && enableBehaviorLogging)
        {
            LogBehavior("✅ PATH CLEAR", "No obstacles detected, resuming normal speed");
        }
        
        isBlocked = false;
    }
    
    void CheckForOtherCars()
    {
        RaycastHit hit;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        
        if (Physics.Raycast(rayStart, transform.forward, out hit, detectionRange))
        {
            RealisticCar otherCar = hit.collider.GetComponent<RealisticCar>();
            if (otherCar != null)
            {
                float relativeSpeed = Vector3.Dot(rb.linearVelocity - otherCar.rb.linearVelocity, transform.forward);
                
                if (relativeSpeed > 2f && hit.distance < detectionRange * 0.7f)
                {
                    isBlocked = true;
                }
            }
        }
    }
    
    bool IsSafeToChangeLanes(float steerInput)
    {
        if (!enableLaneChangeSafety) return true;
        if (Mathf.Abs(steerInput) < 0.3f) return true;
        
        Vector3 steerDirection;
        string targetLane;
        
        if (steerInput > 0.3f)
        {
            steerDirection = transform.right;
            targetLane = "right";
        }
        else if (steerInput < -0.3f)
        {
            steerDirection = -transform.right;
            targetLane = "left";
        }
        else
        {
            return true;
        }
        
        bool isSafe = CheckLaneForCars(steerDirection, targetLane);
        
        if (!isSafe && enableBehaviorLogging && logLaneChanges)
        {
            LogBehavior("🚫 LANE CHANGE BLOCKED", $"Car detected in {targetLane} lane - staying in current lane");
        }
        
        return isSafe;
    }
    
    bool CheckLaneForCars(Vector3 laneDirection, string laneName)
    {
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        float[] checkDistances = { -sideScanDistance * 0.5f, 0f, sideScanDistance * 0.5f };
        
        foreach (float forwardOffset in checkDistances)
        {
            Vector3 checkPosition = rayStart + transform.forward * forwardOffset;
            Vector3 sideRayStart = checkPosition + laneDirection * (laneWidth * 0.5f);
            
            RaycastHit[] hits = Physics.RaycastAll(sideRayStart, Vector3.down, 2f);
            RaycastHit[] horizontalHits = Physics.RaycastAll(sideRayStart, transform.forward, laneChangeSafetyDistance);
            RaycastHit[] backwardHits = Physics.RaycastAll(sideRayStart, -transform.forward, laneChangeSafetyDistance);
            
            List<RaycastHit> allHits = new List<RaycastHit>();
            allHits.AddRange(hits);
            allHits.AddRange(horizontalHits);
            allHits.AddRange(backwardHits);
            
            foreach (RaycastHit hit in allHits)
            {
                RealisticCar otherCar = hit.collider.GetComponent<RealisticCar>();
                if (otherCar != null && otherCar != this)
                {
                    float distance = Vector3.Distance(transform.position, otherCar.transform.position);
                    
                    if (distance < laneChangeSafetyDistance)
                    {
                        return false;
                    }
                }
            }
        }
        
        Vector3 targetLaneCenter = transform.position + laneDirection * laneWidth;
        Collider[] nearbyCars = Physics.OverlapSphere(targetLaneCenter, laneChangeSafetyDistance);
        
        foreach (Collider col in nearbyCars)
        {
            RealisticCar otherCar = col.GetComponent<RealisticCar>();
            if (otherCar != null && otherCar != this)
            {
                float distance = Vector3.Distance(transform.position, otherCar.transform.position);
                
                if (distance < laneChangeSafetyDistance)
                {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    void AIController()
    {
        if (currentNodeIndex >= path.Count)
        {
            isMoving = false;
            ApplyBraking(maxBrakeTorque);
            
            if (enableBehaviorLogging)
            {
                LogBehavior("🏁 DESTINATION REACHED", $"Arrived at {targetEndNode.name}");
            }
            
            Destroy(gameObject, despawnDelay);
            return;
        }
        
        // Get target position
        Vector3 targetPos = GetRoadConstrainedTarget();
        Vector3 localTarget = transform.InverseTransformPoint(targetPos);
        
        // Calculate steering
        float steerInput = 0f;
        string steerReason = "No steering needed";
        
        if (localTarget.magnitude > 0.1f)
        {
            steerInput = Mathf.Clamp(localTarget.x / localTarget.magnitude, -1f, 1f);
            steerReason = $"Steering toward {targetPos} (local: {localTarget})";
            
            if (aggressiveTurning)
            {
                float maxSteerChange = maxSteerAnglePerFrame * Time.deltaTime;
                float steerChange = steerInput - lastSteerAngle;
                steerChange = Mathf.Clamp(steerChange, -maxSteerChange, maxSteerChange);
                steerInput = lastSteerAngle + steerChange;
                lastSteerAngle = steerInput;
                
                steerInput = Mathf.Sign(steerInput) * Mathf.Pow(Mathf.Abs(steerInput), 0.6f);
                steerReason += " (aggressive mode)";
            }
        }
        
        float distanceToCurrentNode = Vector3.Distance(transform.position, path[currentNodeIndex].transform.position);
        
        // Speed decisions
        bool isOnStraightRoad = enableSpeedBoost && IsRoadStraight();
        float speedMultiplier = 1f;
        List<string> speedReasons = new List<string>();
        
        if (isOnStraightRoad)
        {
            speedMultiplier *= straightRoadSpeedMultiplier;
            speedReasons.Add($"Speed boost: {straightRoadSpeedMultiplier}x (straight road)");
        }
        
        if (isOffRoad)
        {
            speedMultiplier *= 0.2f;
            speedReasons.Add("Off-road penalty: 0.2x");
        }
        
        if (nearbyIntersectionNodes.Count > 0 && isAtIntersection)
        {
            speedMultiplier *= intersectionSlowdownFactor;
            speedReasons.Add($"Intersection slowdown: {intersectionSlowdownFactor}x");
        }
        
        if (shouldYieldAtIntersection)
        {
            speedMultiplier = 0f;
            speedReasons.Add("Full stop: Yielding at intersection");
        }
        
        float turnSharpness = Mathf.Abs(steerInput);
        if (turnSharpness > 0.6f)
        {
            float turnPenalty = Mathf.Lerp(1f, turnSlowdownFactor, (turnSharpness - 0.6f) / 0.4f);
            speedMultiplier *= turnPenalty;
            speedReasons.Add($"Sharp turn slowdown: {turnPenalty:F2}x (turn sharpness: {turnSharpness:F2})");
        }
        
        if (distanceToCurrentNode < stoppingDistance * 1.2f)
        {
            float approachPenalty = Mathf.Clamp01(distanceToCurrentNode / (stoppingDistance * 1.2f));
            speedMultiplier *= approachPenalty;
            speedReasons.Add($"Approach slowdown: {approachPenalty:F2}x (distance: {distanceToCurrentNode:F1}m)");
        }
        
        float motorInput = (isBlocked || shouldYieldAtIntersection) ? 0f : speedMultiplier;
        
        if (isBlocked)
        {
            speedReasons.Add("BLOCKED: Motor input = 0");
        }
        
        // Apply controls
        ApplySteering(steerInput);
        ApplyMotor(motorInput);
        
        // Log periodic decisions
        if (enableBehaviorLogging && Time.time - lastLogTime > loggingInterval)
        {
            LogPeriodicDecisions(steerInput, steerReason, motorInput, speedMultiplier, speedReasons, distanceToCurrentNode);
            lastLogTime = Time.time;
        }
        
        // Check for node progression
        if (distanceToCurrentNode < nodeReachDistance)
        {
            if (enableBehaviorLogging && logPathFollowing)
            {
                string nextNodeName = (currentNodeIndex + 1 < path.Count) ? path[currentNodeIndex + 1].name : "DESTINATION";
                LogBehavior("📍 NODE REACHED", $"Completed {path[currentNodeIndex].name}, heading to {nextNodeName}");
            }
            
            currentNodeIndex++;
        }
    }
    
    void LogPeriodicDecisions(float steerInput, string steerReason, float motorInput, float speedMultiplier, List<string> speedReasons, float distanceToNode)
    {
        if (!enableBehaviorLogging) return;
        
        string status = $"🚗 {carID} STATUS";
        string details = $"Node: {currentNodeIndex}/{path.Count} | Distance: {distanceToNode:F1}m | ";
        details += $"Speed: {speedMultiplier:F2}x | Steering: {steerInput:F2} | Motor: {motorInput:F2}";
        
        if (logSteeringDecisions && Mathf.Abs(steerInput) > 0.1f)
        {
            LogBehavior("🎯 STEERING", steerReason + $" (input: {steerInput:F2})");
        }
        
        if (logSpeedDecisions && speedReasons.Count > 0)
        {
            LogBehavior("⚡ SPEED", string.Join(" | ", speedReasons));
        }
        
        LogBehavior(status, details);
    }
    
    Vector3 GetRoadConstrainedTarget()
    {
        if (currentNodeIndex >= path.Count) return transform.position;
        
        Vector3 currentNodePos = path[currentNodeIndex].transform.position;
        float distanceToCurrent = Vector3.Distance(transform.position, currentNodePos);
        
        if (distanceToCurrent < nodeReachDistance * 1.5f && currentNodeIndex + 1 < path.Count)
        {
            return path[currentNodeIndex + 1].transform.position;
        }
        
        return currentNodePos;
    }
    
    bool IsRoadStraight()
    {
        if (path == null || currentNodeIndex >= path.Count - 2) return false;
        
        int lookAheadNodes = Mathf.Min(4, path.Count - currentNodeIndex - 1);
        if (lookAheadNodes < 2) return false;
        
        Vector3 currentDirection = transform.forward;
        float totalStraightDistance = 0f;
        bool isStraight = true;
        
        for (int i = currentNodeIndex; i < currentNodeIndex + lookAheadNodes - 1; i++)
        {
            if (i + 1 >= path.Count) break;
            
            Vector3 segmentStart = path[i].transform.position;
            Vector3 segmentEnd = path[i + 1].transform.position;
            Vector3 segmentDirection = (segmentEnd - segmentStart).normalized;
            
            float angle = Vector3.Angle(currentDirection, segmentDirection);
            
            if (angle > turnDetectionAngle)
            {
                isStraight = false;
                break;
            }
            
            totalStraightDistance += Vector3.Distance(segmentStart, segmentEnd);
            currentDirection = segmentDirection;
        }
        
        bool hasEnoughDistance = totalStraightDistance >= straightRoadMinDistance;
        bool noIntersectionAhead = nearbyIntersectionNodes.Count == 0;
        
        return isStraight && hasEnoughDistance && noIntersectionAhead;
    }
    
    void RecalculatePath()
    {
        if (targetEndNode == null) return;
        
        Node nearestNode = FindNearestNode();
        if (nearestNode == null) return;
        
        List<Node> newPath = PathFinder.FindPath(nearestNode, targetEndNode);
        
        if (newPath != null && newPath.Count > 0)
        {
            path = newPath;
            currentNodeIndex = 0;
            
            if (enableBehaviorLogging && logPathFollowing)
            {
                LogBehavior("🔄 PATH RECALCULATED", $"New route with {newPath.Count} nodes from {nearestNode.name}");
            }
        }
    }
    
    Node FindNearestNode()
    {
        Node[] allNodes = FindObjectsOfType<Node>();
        Node nearestNode = null;
        float nearestDistance = float.MaxValue;
        
        foreach (Node node in allNodes)
        {
            if (node == null) continue;
            
            float distance = Vector3.Distance(transform.position, node.transform.position);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestNode = node;
            }
        }
        
        return nearestNode;
    }
    
    void ApplySteering(float steerInput)
    {
        if (frontLeftWheel == null || frontRightWheel == null) return;
        
        if (!ShouldAllowRotationControl())
        {
            frontLeftWheel.steerAngle = 0;
            frontRightWheel.steerAngle = 0;
            return;
        }
        
        float originalSteerInput = steerInput;
        
        if (enableLaneChangeSafety && !IsSafeToChangeLanes(steerInput))
        {
            steerInput *= 0.1f;
            
            if (enableBehaviorLogging && logLaneChanges && Mathf.Abs(originalSteerInput - steerInput) > 0.1f)
            {
                LogBehavior("🚫 STEERING LIMITED", $"Reduced from {originalSteerInput:F2} to {steerInput:F2} for safety");
            }
        }
        
        float steerAngle = steerInput * maxSteerAngle;
        frontLeftWheel.steerAngle = steerAngle;
        frontRightWheel.steerAngle = steerAngle;
    }
    
    void ApplyMotor(float motorInput)
    {
        if (rearLeftWheel == null || rearRightWheel == null) return;
        
        float motor = motorInput * maxMotorTorque;
        
        rearLeftWheel.motorTorque = motor;
        rearRightWheel.motorTorque = motor;
        
        if (motorInput == 0 || isBlocked)
        {
            ApplyBraking(maxBrakeTorque * 0.7f);
        }
        else
        {
            ApplyBraking(0);
        }
    }
    
    bool ShouldAllowRotationControl()
    {
        return !hasInitialAlignment || alignmentTime > 0.3f;
    }
    
    void ApplyBraking(float brakeForce)
    {
        WheelCollider[] wheels = { frontLeftWheel, frontRightWheel, rearLeftWheel, rearRightWheel };
        
        foreach (WheelCollider wheel in wheels)
        {
            if (wheel != null)
            {
                wheel.brakeTorque = brakeForce;
            }
        }
    }
    
    void UpdateWheelMeshes()
    {
        UpdateWheelMesh(frontLeftWheel, frontLeftMesh);
        UpdateWheelMesh(frontRightWheel, frontRightMesh);
        UpdateWheelMesh(rearLeftWheel, rearLeftMesh);
        UpdateWheelMesh(rearRightWheel, rearRightMesh);
    }
    
    void UpdateWheelMesh(WheelCollider wheelCollider, Transform wheelMesh)
    {
        if (wheelCollider == null || wheelMesh == null) return;
        
        Vector3 position;
        Quaternion rotation;
        
        wheelCollider.GetWorldPose(out position, out rotation);
        
        wheelMesh.position = position;
        wheelMesh.rotation = rotation;
    }
    
    /// <summary>
    /// 🔍 LOGGING SYSTEM - Central logging method
    /// </summary>
    void LogBehavior(string category, string details)
    {
        if (!enableBehaviorLogging) return;
        
        // Use the file-based logger if available
        CarBehaviorLogger.Log(carID, category, details, transform.position);
    }
    
    void OnDrawGizmos()
    {
        // All existing gizmo drawing code remains the same
        Gizmos.color = isBlocked ? Color.red : Color.green;
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
        Gizmos.DrawRay(rayStart, transform.forward * detectionRange);
        
        if (enableLaneChangeSafety)
        {
            Gizmos.color = Color.blue;
            Vector3 leftLaneCenter = transform.position - transform.right * laneWidth;
            Vector3 rightLaneCenter = transform.position + transform.right * laneWidth;
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(leftLaneCenter, laneChangeSafetyDistance * 0.5f);
            
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(rightLaneCenter, laneChangeSafetyDistance * 0.5f);
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(rayStart, transform.right * sideScanDistance);
            Gizmos.DrawRay(rayStart, -transform.right * sideScanDistance);
        }
        
        if (isOffRoad)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, roadWidthTolerance);
        }
        
        if (enableSpeedBoost && IsRoadStraight())
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(transform.position + Vector3.up * 2f, Vector3.one * 0.5f);
            
            Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
            Gizmos.DrawSphere(transform.position, 1.5f);
        }
        
        if (isMoving && currentNodeIndex < path.Count)
        {
            Vector3 target = GetRoadConstrainedTarget();
            
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(target, 0.8f);
            
            Gizmos.color = Color.white;
            Gizmos.DrawLine(transform.position, target);
        }
        
        if (path != null && path.Count > 1 && currentNodeIndex < path.Count)
        {
            Gizmos.color = Color.blue;
            
            for (int i = currentNodeIndex; i < Mathf.Min(currentNodeIndex + 3, path.Count - 1); i++)
            {
                if (path[i] != null && path[i + 1] != null)
                {
                    Gizmos.DrawLine(path[i].transform.position, path[i + 1].transform.position);
                }
            }
        }
        
        if (nearbyIntersectionNodes.Count > 0)
        {
            Gizmos.color = shouldYieldAtIntersection ? Color.red : new Color(1f, 0.5f, 0f);
            Gizmos.DrawWireSphere(transform.position, intersectionDetectionRange);
        }
        
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, otherCarDetectionRange);
    }
}