from .map_utils import Map
import math


class Agent():
    def __init__(self, threshold, step_size, initial_pose = None) -> None:
        
        if (initial_pose is not None):
            self.pose = initial_pose
        else:
            self.pose = {
                'x': 0,
                'y': 0,
                'theta': 0,
                'forward': 0
            }

        self.path = []

        self.polygon = []

        self.step = 0

        self.step_size = step_size

        self.threshold = threshold

    def measure_rssi(self, map: Map):
        return map.rssi_from_current_pose(self.pose)

    # Rotates the directional antenna by a relative amount.
    def rotate(self, theta):
        self.pose['theta'] += theta
        self.path.append(self.pose)

    # Rotates the directional antenna to a specified angle.
    def rotate_absolute(self, theta):
        self.pose['theta'] = theta
        self.path.append(self.pose)

    # Translates the robot by a relative x and y amount.
    def translate(self, x, y):
        self.pose['x'] += x
        self.pose['y'] += y
        self.pose = {
            'x': self.pose['x'] + x,
            'y': self.pose['y'] + y,
            'forward': self.pose['forward'],
            'theta': self.pose['theta'],
        }
        self.path.append(self.pose)

    # Changes the robots notion of "forward," such that it will translate in a different direction upon the next step.
    def orient(self, theta):
        self.pose['forward'] += theta

    # Increases agent step counter and translates the robot by a predefined step size. 
    def take_step(self):
        x = math.cos(self.pose['forward']) * self.step_size
        y = math.sin(self.pose['forward']) * self.step_size
        print(x)
        print(y)
        self.translate(x, y)
        self.step += 1

    # Appends a position to the agent's tracked polygon.
    def append_polygon(self):
        self.polygon.append(self.pose)

    def get_polygon(self):
        return self.polygon

    def get_pose(self):
        return self.pose

    # Returns the angle from the robot on the map compared to position of maximum intensity (jammer source) on that map.
    # NOTE: This currently overlooks how an actual agent would find the angle of maximum intensity by rotating its antenna back and forth to find such an angle (kind of like how a radar scanner would).
    def get_maximum_intensity_angle(self, map):
        max_intensity_pose = map.get_maximum_intensity_position()
        return math.atan2(max_intensity_pose['y'] - self.pose['y'], max_intensity_pose['x'] - self.pose['x'])

    # Checks to see if the robot has completed a full loop around the polygon. 
    def check_polygon_complete(self):
        if (self.step > 1 and len(self.polygon) > 2):
            # If the first and final points are within one step of one another (an epsilon), then consider the polygon to be completed.
            polygon_satisfied = self.polygon[0]['x'] < self.polygon[-1]['x'] + self.step_size and self.polygon[0]['x'] > self.polygon[-1]['x'] - self.step_size and self.polygon[0]['y'] < self.polygon[-1]['y'] + self.step_size and self.polygon[0]['y'] > self.polygon[-1]['y'] - self.step_size
            
            if (polygon_satisfied):
                return True
            else:
                return False
        else:
            return False