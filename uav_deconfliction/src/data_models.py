from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class Waypoint:
    """
    Represents a single point in 3D space with an associated timestamp.
    
    Attributes:
        x (float): X coordinate in meters
        y (float): Y coordinate in meters
        z (float): Z coordinate (altitude) in meters
        timestamp (float): Time in seconds from mission start
    """
    x: float
    y: float
    z: float
    timestamp: float

    def __repr__(self) -> str:
        """String representation of the Waypoint for debugging."""
        return f"Waypoint(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, t={self.timestamp:.2f})"


class Mission:
    """
    Represents a UAV flight mission consisting of a sequence of waypoints.
    The mission trajectory is defined as a continuous path connecting these waypoints.
    
    Attributes:
        mission_id (str): Unique identifier for the mission
        waypoints (List[Waypoint]): Ordered list of waypoints
        start_time (float): Start time of the mission (timestamp of the first waypoint)
        end_time (float): End time of the mission (timestamp of the last waypoint)
    """
    
    def __init__(self, mission_id: str, waypoints: List[Waypoint]):
        """
        Initialize a new Mission. The waypoints are assumed to be provided in order,
        defining the start and end of the mission.

        Args:
            mission_id: Unique string ID
            waypoints: List of Waypoint objects, sorted by time.

        Raises:
            ValueError: If fewer than 2 waypoints are provided.
        """
        if len(waypoints) < 2:
            raise ValueError("A mission must have at least 2 waypoints to define a trajectory.")

        self.mission_id: str = mission_id
        self.waypoints: List[Waypoint] = waypoints
        
        # Derive start and end times from the ordered waypoints
        self.start_time: float = waypoints[0].timestamp
        self.end_time: float = waypoints[-1].timestamp
        
        # Optional sanity check for ordering (can remain as a safeguard)
        for i in range(len(waypoints) - 1):
            if waypoints[i].timestamp > waypoints[i+1].timestamp:
                raise ValueError(f"Waypoints must be sorted by timestamp. Found unsorted at index {i}.")

    def get_duration(self) -> float:
        """
        Calculate the total duration of the mission based on waypoints.
        
        Returns:
            float: Duration in seconds.
        """
        return self.end_time - self.start_time

    def get_segment(self, index: int) -> Tuple[Waypoint, Waypoint]:
        """
        Get a flight segment defined by two consecutive waypoints.

        Args:
            index: The index of the first waypoint in the segment.

        Returns:
            Tuple[Waypoint, Waypoint]: The waypoints at index and index+1.

        Raises:
            IndexError: If the index is out of range for creating a segment.
        """
        if index < 0 or index >= len(self.waypoints) - 1:
            raise IndexError("Segment index out of range")
        return (self.waypoints[index], self.waypoints[index+1])

    def get_all_segments(self) -> List[Tuple[Waypoint, Waypoint]]:
        """
        Get all consecutive waypoint pairs in the mission.

        Returns:
            List[Tuple[Waypoint, Waypoint]]: List of all segments.
        """
        return [self.get_segment(i) for i in range(len(self.waypoints) - 1)]


@dataclass
class Conflict:
    """
    Represents a detected conflict between two UAV missions.
    
    Attributes:
        mission_ids (Tuple[str, str]): IDs of the two conflicting missions
        conflict_location (Tuple[float, float, float]): (x, y, z) coordinates of conflict
        conflict_time (float): Time when the conflict occurs
        separation_distance (float): Measured distance between drones at closest approach
        safety_buffer (float): Minimum required separation distance
        description (str): Human-readable details
    """
    mission_ids: Tuple[str, str]
    conflict_location: Tuple[float, float, float]
    conflict_time: float
    separation_distance: float
    safety_buffer: float
    description: str

    def is_violation(self) -> bool:
        """
        Check if the conflict is a safety violation.

        Returns:
            bool: True if separation distance is less than safety buffer.
        """
        return self.separation_distance < self.safety_buffer

    def get_violation_margin(self) -> float:
        """
        Calculate how much the safety buffer was violated.

        Returns:
            float: Positive value indicating violation amount, or negative if safe.
        """
        return self.safety_buffer - self.separation_distance
