import json
import numpy as np
from typing import Tuple, List, Optional, Dict

class TrackUtils:
    def __init__(self, boundary_file: str = 'track_boundaries.json'):
        self.boundary_file = boundary_file
        self.boundaries = None
        self.load_boundaries()
    
    def load_boundaries(self) -> bool:
        """Load track boundaries from file"""
        try:
            if self.boundary_file and os.path.exists(self.boundary_file):
                with open(self.boundary_file, 'r') as f:
                    self.boundaries = json.load(f)
                return True
        except Exception as e:
            print(f"Error loading boundaries: {e}")
        return False
    
    def get_boundary_points(self) -> Optional[Dict[str, List[int]]]:
        """Get the current boundary points"""
        return self.boundaries
    
    def is_point_in_track(self, point: Tuple[int, int], margin: int = 30) -> bool:
        """Check if a point is within the track boundaries"""
        if not self.boundaries:
            return True  # If no boundaries set, allow all points
            
        x, y = point
        
        # Get boundary points with margin
        tl = self.boundaries["top_left"]
        tr = self.boundaries["top_right"]
        br = self.boundaries["bottom_right"]
        bl = self.boundaries["bottom_left"]
        
        # Add margin to create inner boundary
        tl = [tl[0] + margin, tl[1] + margin]
        tr = [tr[0] - margin, tr[1] + margin]
        br = [br[0] - margin, br[1] - margin]
        bl = [bl[0] + margin, bl[1] - margin]
        
        # Check if point is within the rectangle
        return (tl[0] <= x <= tr[0] and  # Check x bounds
                tl[1] <= y <= bl[1])     # Check y bounds
    
    def get_track_center(self) -> Optional[Tuple[int, int]]:
        """Get the center point of the track"""
        if not self.boundaries:
            return None
            
        # Calculate center as average of all corners
        corners = [
            self.boundaries["top_left"],
            self.boundaries["top_right"],
            self.boundaries["bottom_right"],
            self.boundaries["bottom_left"]
        ]
        
        center_x = sum(c[0] for c in corners) // 4
        center_y = sum(c[1] for c in corners) // 4
        
        return (center_x, center_y)
    
    def get_goal_positions(self, distance_from_edge: int = 100) -> Dict[str, Tuple[int, int]]:
        """Get suggested goal positions based on track boundaries"""
        if not self.boundaries:
            return {}
            
        # Calculate goal positions near the middle of each side
        goals = {}
        
        # Left goal (middle of left side)
        left_mid = (
            self.boundaries["top_left"][0] + distance_from_edge,
            (self.boundaries["top_left"][1] + self.boundaries["bottom_left"][1]) // 2
        )
        goals["left"] = left_mid
        
        # Right goal (middle of right side)
        right_mid = (
            self.boundaries["top_right"][0] - distance_from_edge,
            (self.boundaries["top_right"][1] + self.boundaries["bottom_right"][1]) // 2
        )
        goals["right"] = right_mid
        
        return goals
    
    def get_left_side_target(self) -> Optional[Tuple[int, int]]:
        """Get target position that is 1/6 from the left side of the track"""
        if not self.boundaries:
            return None
            
        # Get left side points
        top_left = self.boundaries["top_left"]
        bottom_left = self.boundaries["bottom_left"]
        
        # Calculate middle point of left side
        mid_y = (top_left[1] + bottom_left[1]) // 2
        
        # Calculate point that is 1/6 from left side
        # First get total width of left side
        left_width = top_left[0] - bottom_left[0]  # This might be negative depending on coordinate system
        target_x = top_left[0] - (abs(left_width) // 6)  # Move 1/6 from left edge
        
        return (target_x, mid_y)
    
    def draw_track_overlay(self, frame: np.ndarray, margin: int = 30) -> None:
        """Draw track boundaries and margin on the frame"""
        if not self.boundaries:
            return
            
        # Draw main boundaries
        points = [
            self.boundaries["top_left"],
            self.boundaries["top_right"],
            self.boundaries["bottom_right"],
            self.boundaries["bottom_left"]
        ]
        cv2.polylines(frame, [np.array(points)], True, (0, 255, 0), 2)
        
        # Draw margin boundary
        margin_points = [
            [self.boundaries["top_left"][0] + margin, self.boundaries["top_left"][1] + margin],
            [self.boundaries["top_right"][0] - margin, self.boundaries["top_right"][1] + margin],
            [self.boundaries["bottom_right"][0] - margin, self.boundaries["bottom_right"][1] - margin],
            [self.boundaries["bottom_left"][0] + margin, self.boundaries["bottom_left"][1] - margin]
        ]
        cv2.polylines(frame, [np.array(margin_points)], True, (0, 0, 255), 1)
        
        # Draw center point
        center = self.get_track_center()
        if center:
            cv2.circle(frame, center, 5, (255, 0, 0), -1)
            cv2.putText(frame, "Center", (center[0]+10, center[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# Example usage:
if __name__ == "__main__":
    import cv2
    import os
    
    # Test the track utilities
    track = TrackUtils()
    
    if not track.boundaries:
        print("No track boundaries found. Please run track_calibration.py first.")
        exit()
    
    # Print track info
    print("\nTrack Information:")
    print("Boundary points:", track.boundaries)
    print("Track center:", track.get_track_center())
    print("Goal positions:", track.get_goal_positions())
    
    # Test visualization
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open camera")
        exit()
    
    print("\nPress 'q' to quit visualization")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Draw track overlay
        track.draw_track_overlay(frame)
        
        cv2.imshow("Track Visualization", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows() 