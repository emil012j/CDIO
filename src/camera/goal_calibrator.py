"""
Simpel goal position click-calibration med delivery og goal positioner
"""

import cv2
import json

class GoalCalibrator:
    def __init__(self):
        self.goal_file = "goal_positions.json"
        self.delivery_position = None
        self.goal_position = None
        self.calibration_step = "delivery"  # "delivery" or "goal"
        
    def mouse_callback(self, event, x, y, flags, param):
        """Click to set delivery and goal positions"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.calibration_step == "delivery":
                self.delivery_position = (x, y)
                self.calibration_step = "goal"
                print("Delivery position sat til: ({}, {}) - klik nu på goal position".format(x, y))
            elif self.calibration_step == "goal":
                self.goal_position = (x, y)
                print("Goal position sat til: ({}, {}) - tryk 'q' for at gemme".format(x, y))

    def save_positions(self):
        """Gem delivery og goal positioner"""
        if self.delivery_position and self.goal_position:
            # Convert tuples to lists for JSON serialization
            delivery_list = [self.delivery_position[0], self.delivery_position[1]]
            goal_list = [self.goal_position[0], self.goal_position[1]]
            data = {
                'calibrated': True, 
                'delivery_position': delivery_list,
                'goal_position': goal_list
            }
            with open(self.goal_file, 'w') as f:
                json.dump(data, f, indent=2)  # Pretty format for readability
            print("Positioner gemt: Delivery({}, {}) -> Goal({}, {}) -> {}".format(
                self.delivery_position[0], self.delivery_position[1],
                self.goal_position[0], self.goal_position[1], self.goal_file))
            return True
        return False

    def start_calibration(self, frame):
        """Start click calibration med delivery og goal positioner"""
        print("=== GOAL CALIBRATION ===")
        print("1. Klik på delivery position (hvor robotten skal køre hen først)")
        print("2. Klik på goal position (hvor robotten skal aflevere bolde)")
        print("3. Tryk 'q' for at gemme, 'ESC' for at annullere")
        
        cv2.namedWindow("Goal Calibration")
        cv2.setMouseCallback("Goal Calibration", self.mouse_callback)
        
        # Reset calibration state
        self.delivery_position = None
        self.goal_position = None
        self.calibration_step = "delivery"
        
        # Calibration loop
        while True:
            display_frame = frame.copy()
            
            # Draw current positions if set
            if self.delivery_position:
                cv2.circle(display_frame, self.delivery_position, 15, (0, 255, 255), -1)  # GUL for delivery
                cv2.putText(display_frame, "DELIVERY: ({},{})".format(self.delivery_position[0], self.delivery_position[1]), 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            if self.goal_position:
                cv2.circle(display_frame, self.goal_position, 15, (0, 255, 0), -1)  # GRØN for goal
                cv2.putText(display_frame, "GOAL: ({},{})".format(self.goal_position[0], self.goal_position[1]), 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw line between delivery and goal
                if self.delivery_position and self.goal_position:
                    cv2.line(display_frame, self.delivery_position, self.goal_position, (255, 255, 255), 2)
                cv2.putText(display_frame, "Press 'q' to save, ESC to cancel", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                if self.delivery_position:
                    cv2.putText(display_frame, "Click to set goal position", (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                else:
                    cv2.putText(display_frame, "Click to set delivery position", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_frame, "ESC to cancel", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.putText(display_frame, "Goal Calibration", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Goal Calibration", display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Save positions
                if self.delivery_position and self.goal_position:
                    if self.save_positions():
                        print("Goal calibration successfully saved!")
                        break
                    else:
                        print("Failed to save goal calibration!")
                else:
                    print("Both delivery and goal positions must be set!")
            elif key == 27:  # ESC key - cancel
                print("Goal calibration cancelled")
                break
        
        cv2.destroyWindow("Goal Calibration")

# For backward compatibility
TrackCalibrator = GoalCalibrator 