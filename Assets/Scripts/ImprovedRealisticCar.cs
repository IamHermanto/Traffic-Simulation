using System.Collections.Generic;
using UnityEngine;

public class ImprovedRealisticCar : MonoBehaviour
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
    public float cruiseSpeed = 15f; // Target speed in m/s
    
    [Header("Lane Change Settings")]
    public float laneChangeAnticipationDistance = 20f; // How far ahead to plan lane changes
    public float laneChangeSpeed = 5f; // Speed when changing lanes
    public float minGapForLaneChange = 5f; // Minimum gap needed to change lanes
    public float laneChangeTimeout = 10f; // Max time to wait for lane change
    
    [Header("Steering Control")]
    public float steeringSmoothTime = 0.1f; // Smoothing for steering input
    public float maxSteerChangeRate = 2f; // Max steering change per second
    public float lookAheadDistance = 5f; // How far ahead to look for path following
    public float pathFollowingGain = 2f; // Gain for path following controller
    
    [Header("Speed Control")]
    public float speedSmoothTime = 0.5f; // Smoothing for speed changes
    public float followDistance = 4f; // Distance to maintain behind other cars
    public float intersectionSlowdownDistance = 10f;
    public float turnSlowdownFactor = 0.5f;
    
    [Header("AI Pathfinding")]
    public List<Node> path = new List<Node>();
    public int currentNodeIndex = 0;
    public float nodeReachDistance = 2f;
    
    // Core components
    private Rigidbody rb;
    
    // State management
    private enum DrivingState
    {
        Following,          // Following the path normally
        PreparingLaneChange, // Slowing down to create gap for lane change
        ChangingLanes,      // Actively changing lanes
        Yielding,          // Yielding at intersection
        Stopping           // Coming to a stop
    }
    
    private DrivingState currentState = DrivingState.Following;
    private float stateTimer = 0f;
    
    // Lane change planning
    private bool needsLaneChange = false;
    private int targetLaneDirection = 0; // -1 for left, 1 for right, 0 for no change
    private float laneChangeProgress = 0f;
    private Vector3 laneChangeStartPos;
    private Vector3 laneChangeTargetPos;
    private Node laneChangeTargetNode;
    
    // Smooth control values
    private float currentSteerAngle = 0f;
    private float targetSteerAngle = 0f;
    private float steerVelocity = 0f;
    public float currentSpeed = 0f;  // Made public so other cars can read it
    private float targetSpeed = 0f;
    private float speedVelocity = 0f;
    
    // Detection
    private MonoBehaviour carInFront = null;  // Can be either RealisticCar or ImprovedRealisticCar
    private MonoBehaviour carBeside = null;   // Can be either RealisticCar or ImprovedRealisticCar
    private float distanceToCarInFront = float.MaxValue;
    private float distanceToCarBeside = float.MaxValue;
    
    // Path analysis
    private string currentLane = "";
    private string requiredLane = "";
    private float distanceToLaneChangePoint = float.MaxValue;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0.5f);
        
        SetupWheelFriction();
        currentSpeed = 0f;
        targetSpeed = cruiseSpeed;
    }
    
    void Update()
    {
        if (path != null && path.Count > 0)
        {
            // Update state timer
            stateTimer += Time.deltaTime;
            
            // Analyze the path and surroundings
            AnalyzePath();
            DetectNearbyVehicles();
            
            // State machine for AI behavior
            UpdateStateMachine();
            
            // Apply control inputs
            ApplySmoothedControls();
        }
        
        UpdateWheelMeshes();
    }
    
    void UpdateStateMachine()
    {
        switch (currentState)
        {
            case DrivingState.Following:
                HandleFollowingState();
                break;
                
            case DrivingState.PreparingLaneChange:
                HandlePreparingLaneChangeState();
                break;
                
            case DrivingState.ChangingLanes:
                HandleChangingLanesState();
                break;
                
            case DrivingState.Yielding:
                HandleYieldingState();
                break;
                
            case DrivingState.Stopping:
                HandleStoppingState();
                break;
        }
        
        // Check if we've reached the current node
        if (currentNodeIndex < path.Count)
        {
            float distToNode = Vector3.Distance(transform.position, path[currentNodeIndex].transform.position);
            if (distToNode < nodeReachDistance)
            {
                currentNodeIndex++;
                if (currentNodeIndex >= path.Count)
                {
                    TransitionToState(DrivingState.Stopping);
                }
            }
        }
    }
    
    void HandleFollowingState()
    {
        // Check if we need to prepare for a lane change
        if (needsLaneChange && distanceToLaneChangePoint < laneChangeAnticipationDistance)
        {
            TransitionToState(DrivingState.PreparingLaneChange);
            return;
        }
        
        // Normal path following
        CalculatePathFollowingSteering();
        
        // Speed control based on conditions
        if (carInFront != null && distanceToCarInFront < followDistance * 2)
        {
            // Match speed of car in front
            float otherCarSpeed = GetCarSpeed(carInFront);
            targetSpeed = Mathf.Min(otherCarSpeed * 0.9f, cruiseSpeed);
        }
        else if (IsApproachingTurn())
        {
            targetSpeed = cruiseSpeed * turnSlowdownFactor;
        }
        else
        {
            targetSpeed = cruiseSpeed;
        }
    }
    
    void HandlePreparingLaneChangeState()
    {
        // Slow down to create a gap for lane change
        if (carBeside != null && distanceToCarBeside < minGapForLaneChange * 2)
        {
            // Slow down significantly to let the car beside pass
            float otherCarSpeed = GetCarSpeed(carBeside);
            targetSpeed = otherCarSpeed * 0.5f;
            
            // Keep following current path while slowing down
            CalculatePathFollowingSteering();
            
            // Check if gap is now sufficient
            if (distanceToCarBeside > minGapForLaneChange * 2 || carBeside == null)
            {
                // Gap created, initiate lane change
                InitiateLaneChange();
                TransitionToState(DrivingState.ChangingLanes);
            }
        }
        else
        {
            // No car beside or gap is already sufficient
            InitiateLaneChange();
            TransitionToState(DrivingState.ChangingLanes);
        }
        
        // Timeout check
        if (stateTimer > laneChangeTimeout)
        {
            // Taking too long, force the lane change or abort
            Debug.LogWarning($"{gameObject.name}: Lane change timeout, forcing change");
            InitiateLaneChange();
            TransitionToState(DrivingState.ChangingLanes);
        }
    }
    
    void HandleChangingLanesState()
    {
        // Execute smooth lane change
        laneChangeProgress += Time.deltaTime / 2f; // 2 seconds for lane change
        laneChangeProgress = Mathf.Clamp01(laneChangeProgress);
        
        // Smooth interpolation for lane change
        Vector3 targetPosition = Vector3.Lerp(laneChangeStartPos, laneChangeTargetPos, 
            Mathf.SmoothStep(0, 1, laneChangeProgress));
        
        // Calculate steering to reach target position
        Vector3 toTarget = targetPosition - transform.position;
        Vector3 localTarget = transform.InverseTransformDirection(toTarget);
        targetSteerAngle = Mathf.Clamp(localTarget.x * pathFollowingGain, -1f, 1f) * maxSteerAngle;
        
        // Maintain moderate speed during lane change
        targetSpeed = laneChangeSpeed;
        
        // Check if lane change is complete
        if (laneChangeProgress >= 1f)
        {
            needsLaneChange = false;
            targetLaneDirection = 0;
            TransitionToState(DrivingState.Following);
        }
    }
    
    void HandleYieldingState()
    {
        targetSpeed = 0f;
        targetSteerAngle = 0f;
        
        // Check if intersection is clear
        if (!IsIntersectionBlocked())
        {
            TransitionToState(DrivingState.Following);
        }
    }
    
    void HandleStoppingState()
    {
        targetSpeed = 0f;
        targetSteerAngle = 0f;
        
        if (rb.linearVelocity.magnitude < 0.1f)
        {
            // Car has stopped, despawn after delay
            Destroy(gameObject, 1f);
        }
    }
    
    void CalculatePathFollowingSteering()
    {
        if (currentNodeIndex >= path.Count) return;
        
        // Pure pursuit algorithm for smooth path following
        Vector3 targetPoint = GetLookAheadPoint();
        Vector3 localTarget = transform.InverseTransformPoint(targetPoint);
        
        // Calculate curvature to reach the target point
        float curvature = 2f * localTarget.x / (localTarget.magnitude * localTarget.magnitude);
        
        // Convert curvature to steering angle (use a wheelbase estimate if no collider)
        float wheelbase = 2.5f; // Default wheelbase
        BoxCollider boxCollider = GetComponent<BoxCollider>();
        if (boxCollider != null)
            wheelbase = boxCollider.size.z;
        
        targetSteerAngle = Mathf.Atan(curvature * wheelbase) * Mathf.Rad2Deg;
        targetSteerAngle = Mathf.Clamp(targetSteerAngle, -maxSteerAngle, maxSteerAngle);
    }
    
    Vector3 GetLookAheadPoint()
    {
        // Find the point on the path that's lookAheadDistance away
        float accumulatedDistance = 0f;
        Vector3 lastPoint = transform.position;
        
        for (int i = currentNodeIndex; i < path.Count; i++)
        {
            Vector3 currentPoint = path[i].transform.position;
            float segmentDistance = Vector3.Distance(lastPoint, currentPoint);
            
            if (accumulatedDistance + segmentDistance >= lookAheadDistance)
            {
                // Interpolate to get exact look-ahead point
                float t = (lookAheadDistance - accumulatedDistance) / segmentDistance;
                return Vector3.Lerp(lastPoint, currentPoint, t);
            }
            
            accumulatedDistance += segmentDistance;
            lastPoint = currentPoint;
        }
        
        // If we've reached the end of the path
        return path[path.Count - 1].transform.position;
    }
    
    void AnalyzePath()
    {
        if (currentNodeIndex >= path.Count) return;
        
        // Determine current lane
        currentLane = GetNodeLane(path[currentNodeIndex]);
        
        // Look ahead to see if we need to change lanes
        needsLaneChange = false;
        for (int i = currentNodeIndex + 1; i < Mathf.Min(currentNodeIndex + 10, path.Count); i++)
        {
            string nodeLane = GetNodeLane(path[i]);
            if (nodeLane != currentLane && nodeLane != "Center")
            {
                requiredLane = nodeLane;
                needsLaneChange = true;
                distanceToLaneChangePoint = CalculatePathDistance(currentNodeIndex, i);
                
                // Determine direction of lane change
                if (requiredLane == "Left" && currentLane == "Right")
                    targetLaneDirection = -1;
                else if (requiredLane == "Right" && currentLane == "Left")
                    targetLaneDirection = 1;
                    
                break;
            }
        }
    }
    
    float CalculatePathDistance(int startIndex, int endIndex)
    {
        float distance = 0f;
        for (int i = startIndex; i < endIndex && i < path.Count - 1; i++)
        {
            distance += Vector3.Distance(path[i].transform.position, path[i + 1].transform.position);
        }
        return distance;
    }
    
    void DetectNearbyVehicles()
    {
        carInFront = null;
        carBeside = null;
        distanceToCarInFront = float.MaxValue;
        distanceToCarBeside = float.MaxValue;
        
        // Cast rays and overlap checks to detect nearby vehicles
        RaycastHit hit;
        
        // Check for car in front
        if (Physics.Raycast(transform.position + Vector3.up * 0.5f, transform.forward, out hit, followDistance * 3))
        {
            // Check for either type of car
            MonoBehaviour otherCar = hit.collider.GetComponent<ImprovedRealisticCar>();
            if (otherCar == null)
                otherCar = hit.collider.GetComponent<RealisticCar>();
                
            if (otherCar != null && otherCar != this)
            {
                carInFront = otherCar;
                distanceToCarInFront = hit.distance;
            }
        }
        
        // Check for cars beside (for lane changes)
        if (targetLaneDirection != 0)
        {
            Vector3 sideDirection = targetLaneDirection > 0 ? transform.right : -transform.right;
            
            // Multiple rays for better detection
            for (float offset = -2f; offset <= 2f; offset += 1f)
            {
                Vector3 rayOrigin = transform.position + Vector3.up * 0.5f + transform.forward * offset;
                if (Physics.Raycast(rayOrigin, sideDirection, out hit, 4f))
                {
                    // Check for either type of car
                    MonoBehaviour otherCar = hit.collider.GetComponent<ImprovedRealisticCar>();
                    if (otherCar == null)
                        otherCar = hit.collider.GetComponent<RealisticCar>();
                        
                    if (otherCar != null && otherCar != this)
                    {
                        float dist = Vector3.Distance(transform.position, otherCar.transform.position);
                        if (dist < distanceToCarBeside)
                        {
                            carBeside = otherCar;
                            distanceToCarBeside = dist;
                        }
                    }
                }
            }
        }
    }
    
    void InitiateLaneChange()
    {
        laneChangeProgress = 0f;
        laneChangeStartPos = transform.position;
        
        // Calculate target position for lane change
        float laneWidth = 3.5f; // Adjust based on your road width
        Vector3 laneChangeDirection = targetLaneDirection > 0 ? transform.right : -transform.right;
        laneChangeTargetPos = transform.position + laneChangeDirection * laneWidth;
        
        // Find the best node in the target lane
        Node[] allNodes = FindObjectsOfType<Node>();
        float nearestDist = float.MaxValue;
        foreach (Node node in allNodes)
        {
            if (GetNodeLane(node) == requiredLane)
            {
                float dist = Vector3.Distance(laneChangeTargetPos, node.transform.position);
                if (dist < nearestDist)
                {
                    nearestDist = dist;
                    laneChangeTargetNode = node;
                }
            }
        }
        
        if (laneChangeTargetNode != null)
        {
            laneChangeTargetPos = laneChangeTargetNode.transform.position;
        }
    }
    
    void ApplySmoothedControls()
    {
        // Smooth steering input
        currentSteerAngle = Mathf.SmoothDamp(currentSteerAngle, targetSteerAngle, 
            ref steerVelocity, steeringSmoothTime, maxSteerChangeRate * maxSteerAngle);
        
        // Apply steering to front wheels
        if (frontLeftWheel != null && frontRightWheel != null)
        {
            frontLeftWheel.steerAngle = currentSteerAngle;
            frontRightWheel.steerAngle = currentSteerAngle;
        }
        
        // Smooth speed control
        currentSpeed = Mathf.SmoothDamp(currentSpeed, targetSpeed, 
            ref speedVelocity, speedSmoothTime);
        
        // Calculate motor torque based on speed error
        float currentVelocity = rb.linearVelocity.magnitude;
        float speedError = currentSpeed - currentVelocity;
        float motorTorque = Mathf.Clamp(speedError * 200f, -maxMotorTorque, maxMotorTorque);
        
        // Apply motor torque to rear wheels
        if (rearLeftWheel != null && rearRightWheel != null)
        {
            rearLeftWheel.motorTorque = motorTorque;
            rearRightWheel.motorTorque = motorTorque;
            
            // Apply braking if needed
            float brakeTorque = speedError < -1f ? Mathf.Abs(speedError) * 500f : 0f;
            rearLeftWheel.brakeTorque = brakeTorque;
            rearRightWheel.brakeTorque = brakeTorque;
            frontLeftWheel.brakeTorque = brakeTorque;
            frontRightWheel.brakeTorque = brakeTorque;
        }
    }
    
    void TransitionToState(DrivingState newState)
    {
        if (currentState != newState)
        {
            Debug.Log($"{gameObject.name}: State transition {currentState} -> {newState}");
            currentState = newState;
            stateTimer = 0f;
        }
    }
    
    bool IsApproachingTurn()
    {
        if (currentNodeIndex >= path.Count - 2) return false;
        
        Vector3 currentDir = (path[currentNodeIndex + 1].transform.position - 
                             path[currentNodeIndex].transform.position).normalized;
        Vector3 nextDir = (path[currentNodeIndex + 2].transform.position - 
                          path[currentNodeIndex + 1].transform.position).normalized;
        
        float angle = Vector3.Angle(currentDir, nextDir);
        return angle > 30f;
    }
    
    bool IsIntersectionBlocked()
    {
        // Check for other cars at intersection
        Collider[] nearby = Physics.OverlapSphere(transform.position, 5f);
        foreach (Collider col in nearby)
        {
            if (col.gameObject != gameObject)
            {
                // Check for either type of car
                MonoBehaviour otherCar = col.GetComponent<ImprovedRealisticCar>();
                if (otherCar == null)
                    otherCar = col.GetComponent<RealisticCar>();
                    
                if (otherCar != null)
                {
                    // Check if other car has right of way
                    Vector3 toOther = col.transform.position - transform.position;
                    float angle = Vector3.Angle(transform.right, toOther);
                    if (angle < 45f) // Car is to our right
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    string GetNodeLane(Node node)
    {
        if (node == null) return "Unknown";
        
        string nodeName = node.name.ToLower();
        
        if (nodeName.Contains("left"))
            return "Left";
        if (nodeName.Contains("right"))
            return "Right";
            
        // Check parent
        if (node.transform.parent != null)
        {
            string parentName = node.transform.parent.name.ToLower();
            if (parentName.Contains("left"))
                return "Left";
            if (parentName.Contains("right"))
                return "Right";
        }
        
        return "Center";
    }
    
    // Helper method to get speed from either type of car
    float GetCarSpeed(MonoBehaviour car)
    {
        if (car == null) return 0f;
        
        // Try to get speed from ImprovedRealisticCar
        ImprovedRealisticCar improvedCar = car as ImprovedRealisticCar;
        if (improvedCar != null)
            return improvedCar.currentSpeed;
        
        // Try to get speed from regular RealisticCar using velocity
        RealisticCar regularCar = car as RealisticCar;
        if (regularCar != null)
        {
            Rigidbody carRb = regularCar.GetComponent<Rigidbody>();
            if (carRb != null)
                return carRb.linearVelocity.magnitude;
        }
        
        // Fallback - try to get velocity from any rigidbody
        Rigidbody rb = car.GetComponent<Rigidbody>();
        if (rb != null)
            return rb.linearVelocity.magnitude;
            
        return 0f;
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
            if (wheel != null)
            {
                wheel.forwardFriction = forwardFriction;
                wheel.sidewaysFriction = sidewaysFriction;
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
    
    public void SetPath(List<Node> newPath)
    {
        path = newPath;
        currentNodeIndex = 0;
        currentState = DrivingState.Following;
        
        // Align to initial direction
        if (path != null && path.Count > 1)
        {
            Vector3 initialDir = (path[1].transform.position - path[0].transform.position).normalized;
            transform.rotation = Quaternion.LookRotation(initialDir);
        }
    }
    
    // Compatibility method for existing spawner
    public void SetPathWithoutAlignment(List<Node> newPath)
    {
        SetPath(newPath);
        // The improved version handles alignment better anyway
    }
    
    void OnDrawGizmos()
    {
        // Visualize look-ahead point
        if (Application.isPlaying && path != null && path.Count > 0)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(GetLookAheadPoint(), 0.5f);
            
            // Show current state
            Vector3 statePos = transform.position + Vector3.up * 3f;
            Gizmos.color = GetStateColor();
            Gizmos.DrawCube(statePos, Vector3.one * 0.5f);
            
            // Show lane change target
            if (currentState == DrivingState.ChangingLanes)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position, laneChangeTargetPos);
                Gizmos.DrawWireSphere(laneChangeTargetPos, 0.3f);
            }
            
            // Show detected vehicles
            if (carInFront != null)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(transform.position, carInFront.transform.position);
            }
            
            if (carBeside != null)
            {
                Gizmos.color = new Color(1f, 0.5f, 0f); // Orange color
                Gizmos.DrawLine(transform.position, carBeside.transform.position);
            }
        }
    }
    
    Color GetStateColor()
    {
        switch (currentState)
        {
            case DrivingState.Following: return Color.green;
            case DrivingState.PreparingLaneChange: return Color.yellow;
            case DrivingState.ChangingLanes: return Color.cyan;
            case DrivingState.Yielding: return Color.red;
            case DrivingState.Stopping: return Color.gray;
            default: return Color.white;
        }
    }
}