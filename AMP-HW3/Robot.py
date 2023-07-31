import itertools
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString


class Robot(object):

    def __init__(self):

        # define robot properties
        self.links = np.array([80.0, 70.0, 40.0, 40.0])
        self.dim = len(self.links)

        # robot field of fiew (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi / 3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 60.0

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        return np.linalg.norm(np.array(prev_config) - np.array(next_config))

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links (including end-effector) and return.
        @param given_config Given configuration.
        '''
        pos = np.empty((4, 2))

        x = 0
        y = 0
        angle = 0
        for i in range(4):
            angle = self.compute_link_angle(angle, given_config[i])
            link_len = self.links[i]
            x = x + link_len * np.cos(angle)
            y = y + link_len * np.sin(angle)
            pos[i][0] = x
            pos[i][1] = y

        return pos

    def compute_ee_angle(self, given_config):
        '''
        Compute the 1D orientation of the end-effector w.r.t. world origin (or first joint)
        @param given_config Given configuration.
        '''
        ee_angle = given_config[0]
        for i in range(1, len(given_config)):
            ee_angle = self.compute_link_angle(ee_angle, given_config[i])

        return ee_angle

    def compute_link_angle(self, link_angle, given_angle):
        '''
        Compute the 1D orientation of a link given the previous link and the current joint angle.
        @param link_angle previous link angle.
        @param given_angle Given joint angle.
        '''
        if link_angle + given_angle > np.pi:
            return link_angle + given_angle - 2 * np.pi
        elif link_angle + given_angle < -np.pi:
            return link_angle + given_angle + 2 * np.pi
        else:
            return link_angle + given_angle

    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        links = []
        for i in range(len(robot_positions) - 1):
            next_link = LineString([robot_positions[i], robot_positions[i + 1]])
            # if i == 0:
            #     links.append(next_link)
            #     continue
            for j in range(len(links)):
                # Check that a link doesn't touch any that aren't connected to it
                if j < len(links) - 1 and next_link.intersects(links[j]):
                    return False
                # Check that no two links overlap
                if next_link.overlaps(links[j]):
                    return False

            # The last 2 links are equal in size and therefor overlap returns false
            # if the last 2 links have all points in common.  Check if they are the same coords.
            if i == len(robot_positions) - 2:
                if next_link == links[-1]:
                    return False

            links.append(next_link)

        return True
