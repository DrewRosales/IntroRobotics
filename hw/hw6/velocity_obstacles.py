import signal
import sys

import numpy as np
from typing import List

from time import sleep

from matplotlib import pyplot
from descartes import PolygonPatch

from shapely.ops import unary_union
from shapely.geometry import Point, Polygon
from shapely.geometry.multipolygon import MultiPolygon

EXTRA_BUFFER = 0.05

# This function is included to insure the executable can be exited without issue
def sigint_handler(signal, frame):
    print('Interrupted')
    sys.exit(0)

class Agent:
    """
    This class defines the parameters of a mobile agent that avoids collisions
    """
    position = np.zeros(2)
    radius = 1
    max_speed = 1.5
    min_time = None
    def __init__(self, p, r=0.75, s=1.5, t=None):
        self.position = p
        self.radius = r
        self.max_speed = s
        self.min_time = t
        
class Obstacle:
    """
    This class defines the parameters of an obstacle which moves with constant velocity
    """
    position = np.zeros(2)
    radius = 1
    velocity = np.zeros(2)
    def __init__(self,p,r,v):
        self.position = p
        self.radius = r
        self.velocity = v
    
class VelocityObstacle:
    """
    This class provides the default structure for representing a velocity obstacle. 
    
    The origin is the base of the velocity obstacle. With the left and right vectors
    extending outward from that point to define the cone of invalid velocities. 
    
    The left and right vectors should be unit vectors, as they represent a region.
    
    The origin of the VO is defined in the global coordiate system. 
    example: 
         Given a Velocity Obstacle with  origin of [3,3] a left vector of [0,1] and
         right vector of [1,0] an agent with position [2,3] could not have a velocity
         of [1.5,1] as that would fall inside of the velocity obstacle but a velocity
         of [1.5,-1] would be valid as it is outside of the velocity obstacle
    """
    
    origin = np.zeros(2)
    left_vector =  np.zeros(2)
    right_vector =  np.zeros(2)
    
def velocity_obstacle(agent: Agent, obstacle: Obstacle):
    """
    Computes the velocity obstacle, a cone defined by an origin and a left and right vector. The origin
    of the velocity obstacle is in global space, the left and right vectors are with respect to its origin.
    
    This obstacle must take into account both the size of the obstacle and the size of the agent. 
    
    Hint: Consider using the EXTRA_BUFFER variable to increase the size of the velocity obstacle to 
    reduce the chance of collision due to hugging the obstacle too closely
    
    Worth 50 pts
    
    Input
      :param agent: An instance of the Agent class. The agent that will execute the velocity command
      :param obstacle: A single Obstacle object that will be used to compute the velocity obstacle

    Output
      :return: VO: a VelocityObstacle object representing the region of invalid trajectories in global coordinates
    """

    relative_pose = obstacle.position - agent.position
    radius = agent.radius + obstacle.radius + EXTRA_BUFFER

    if np.linalg.norm(relative_pose):
        return None

    direction = relative_pose / np.linalg.norm(relative_pose)

    tangent = np.array([-direction[1], direction[0]])

    left = (tangent * (radius/relative_pose) + direction)
    left /= np.linalg.norm(left)
    right = tangent * (radius/relative_pose) - direction
    right /= np.linalg.norm(right)

    VO = VelocityObstacle(origin = obstacle.velocity, left_vector=left, right_vector=right)
    return VO

def compute_safe_desired_vel(agent: Agent, goal: np.ndarray, obstacles: List[Obstacle]):
    """
    Computes the safest velocity (outside of any velocity obstacle) that is the closest to the
    desired velocity a vector in the direction of the goal constrained by the agents maximum velocity.
    
    The agent is holonomic and can travel in any direction, but the total length of the velocity vector
    must be less than or equal to the maximum velocity
    
    If no valid velocity is available then the vehicle should have a velocity of [0,0]

    Worth 50 pts
    
    Input
      :param agent: An instance of the Agent class. The agent that will execute the velocity command
      :param goal: A 2 element numpy.ndarray containg the x,y coordinates of the goal position
      :param obstacles: A list of Obstacle objects that exist in the environment and must be avoided

    Output
      :return: safe_vel: a 2 element numpy.ndarray with the x and y components of the velocity
    """

    #TODO Implement this function. 50 pts
    raise NotImplementedError

def extract_coords(polygon):
    """
    A helper class for extracting coordinates from different polygon types in shapely
    """
    if isinstance(polygon, MultiPolygon):
        coords_list = []
        [coords_list.extend(p.exterior.coords[:]) for p in polygon.geoms]
    else:
        coords_list = polygon.exterior.coords
    return coords_list
        
        
def VO_to_geom(obstacle: VelocityObstacle):
    """
    A helper class for converting velocity obstacle objects to Shapely Geometry objects.
    
    The scaling factor was chosen to be arbitrarily large to handle very fast obstacles.
    
    Input
      :param obstacle: The VelocityObstacle object to convert
    Output
      :return: poly: A shapely geometry object useful for visualization and intersection checks 
    """

    scaling_factor = 100
    return Polygon([obstacle.origin, obstacle.origin+scaling_factor*obstacle.right_vector, obstacle.origin+scaling_factor*obstacle.left_vector])

def visualize(agent, goal, obstacles):
    """
    A helper class for visualizing the agent and obstacles.
    """
    goal_geom = Point(goal[0], goal[1]).buffer(0.1)
    agent_geom = Point(agent.position[0], agent.position[1]).buffer(agent.radius)
    agent_vel_geom = Point(agent.position[0], agent.position[1]).buffer(agent.max_speed)

    VOs = [velocity_obstacle(agent,obs) for obs in obstacles]
        
    fig = pyplot.figure(1, figsize=[5, 5], dpi=90)
    ax = fig.add_subplot(111)
    
    
    patch1 = PolygonPatch(goal_geom, fc='black', ec='blue', alpha=0.8, zorder=1)
    ax.add_patch(patch1)

    patch1 = PolygonPatch(agent_vel_geom, fc='white', ec='blue', alpha=0.8, zorder=1)
    ax.add_patch(patch1)
    
    patch1 = PolygonPatch(agent_geom, fc='blue', ec='blue', alpha=0.8, zorder=1)
    ax.add_patch(patch1)

    for obs in obstacles:
        p = Point(obs.position[0],obs.position[1]).buffer(obs.radius)
        patch = PolygonPatch(p, fc='blue', ec='blue', alpha=0.6, zorder=2)
        ax.add_patch(patch)
        vo_geom = VO_to_geom(velocity_obstacle(agent, obs))
        patch = PolygonPatch(vo_geom, fc='grey', ec='grey', alpha=0.6, zorder=2)
        ax.add_patch(patch)
            
    best_vel = compute_safe_desired_vel(agent, goal, obstacles)
    ax.arrow(agent.position[0], agent.position[1], best_vel[0], best_vel[1], head_width=0.05, head_length=0.1, fc='black', ec='black')
    pyplot.xlim([-10, 10])
    pyplot.ylim([-10, 10])
    pyplot.grid(True)
    pyplot.draw()
    pyplot.pause(0.0001)

def main():
    
    """
    A default main class for testing and debugging the algorithm with different starting positions
    and obstacle configurations.
    
    Feel free to edit when testing.
    """

    signal.signal(signal.SIGINT, sigint_handler)

    agent = Agent(np.array([-5.0,-5.0]), r=0.75)
    
    goal = np.array([0.0, 8])

    obstacles = []
    obstacles.append(Obstacle(np.array([1.0,1.5]),0.5,np.array([0,0])))
    obstacles.append(Obstacle(np.array([-2.0,5.5]),0.5,np.array([0,-1])))
    obstacles.append(Obstacle(np.array([8.0,6.5]),0.5,np.array([-1,0])))
    obstacles.append(Obstacle(np.array([-3.0,-2]),0.5,np.array([0,1])))


    dt = 0.1

    for i in range(1000):
        best_vel = compute_safe_desired_vel(agent, goal, obstacles)
        if i % 2 == 0:
            visualize(agent, goal, obstacles)
        print(best_vel)
        agent.position += best_vel * dt
        
        for o in obstacles:
            o.position += o.velocity * dt
            
        print(np.linalg.norm(agent.position-goal))
        if np.linalg.norm(agent.position-goal)<agent.radius:
            break

if __name__ == '__main__':
    main()
