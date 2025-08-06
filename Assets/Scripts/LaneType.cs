using UnityEngine;

[System.Flags]
public enum LaneType
{
    None = 0,
    Straight = 1,
    LeftTurn = 2,
    RightTurn = 4,
    
    // Combinations
    StraightOnly = Straight,
    LeftAndStraight = LeftTurn | Straight,
    RightAndStraight = RightTurn | Straight,
    AllDirections = LeftTurn | Straight | RightTurn
}

// Helper methods
public static class LaneTypeExtensions
{
    public static bool AllowsStraight(this LaneType laneType)
    {
        return (laneType & LaneType.Straight) != 0;
    }
    
    public static bool AllowsLeftTurn(this LaneType laneType)
    {
        return (laneType & LaneType.LeftTurn) != 0;
    }
    
    public static bool AllowsRightTurn(this LaneType laneType)
    {
        return (laneType & LaneType.RightTurn) != 0;
    }
    
    public static string GetDisplayName(this LaneType laneType)
    {
        switch (laneType)
        {
            case LaneType.StraightOnly:
                return "Straight Only";
            case LaneType.LeftAndStraight:
                return "Left + Straight";
            case LaneType.RightAndStraight:
                return "Right + Straight";
            case LaneType.AllDirections:
                return "All Directions";
            default:
                return "No Restrictions";
        }
    }
}