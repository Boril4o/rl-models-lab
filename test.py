import pybullet as p
import pybullet_data
import time
import math
import os

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.resetDebugVisualizerCamera(
    cameraDistance=20,      # Distance from target (meters)
    cameraYaw=50,            # Left/Right angle (degrees)
    cameraPitch=-35,         # Up/Down angle (degrees)
    cameraTargetPosition=[0, 0, 0] # The point you are looking AT (x, y, z)
)

planeId = p.loadURDF("plane.urdf")
ballId = p.loadURDF("sphere2.urdf", [0, 0, 0.5])
cubeId = p.loadURDF("cube.urdf", [2, 0, 0.5])
goalId = p.loadURDF("./models/goal.urdf", [0, 7, 0.05], p.getQuaternionFromEuler([math.radians(90), 0, 0]))

wall_size = [10, 0.5, 10]

wall_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=wall_size,
    rgbaColor=[0, 0, 0, 0]
)

wall_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=wall_size
)

left_wall_id = p.createMultiBody(
    baseMass=100,
    baseCollisionShapeIndex=wall_collision_shape_id,
    baseVisualShapeIndex=wall_visual_shape_id,
    basePosition=[-10, 0, wall_size[2]],
    baseOrientation=p.getQuaternionFromEuler([math.radians(0), math.radians(0), math.radians(90)])
)

right_wall_id = p.createMultiBody(
    baseMass=100,
    baseCollisionShapeIndex=wall_collision_shape_id,
    baseVisualShapeIndex=wall_visual_shape_id,
    basePosition=[10, 0, wall_size[2]],
    baseOrientation=p.getQuaternionFromEuler([math.radians(0), math.radians(0), math.radians(90)])
)

back_wall_id = p.createMultiBody(
    baseMass=100,
    baseCollisionShapeIndex=wall_collision_shape_id,
    baseVisualShapeIndex=wall_visual_shape_id,
    basePosition=[0, -10.5, wall_size[2]],
)

front_wall_id = p.createMultiBody(
    baseMass=100,
    baseCollisionShapeIndex=wall_collision_shape_id,
    baseVisualShapeIndex=wall_visual_shape_id,
    basePosition=[0, 10.5, wall_size[2]],
)

p.changeVisualShape(front_wall_id, -1, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(right_wall_id, -1, rgbaColor=[0, 0, 0, 0])


p.changeDynamics(planeId, -1, restitution=0.9)

p.changeDynamics(ballId, -1, restitution=0.9, lateralFriction=0.1, mass=0.3) 

p.changeDynamics(cubeId, -1, mass=10)

cooldown = 0 

p.resetBaseVelocity(cubeId, [-5, 0, 0])

while p.isConnected():
    p.stepSimulation()
    
    contact_points = p.getContactPoints(bodyA=cubeId, bodyB=ballId)
    
    if len(contact_points) > 0 and cooldown == 0:

        cubePos, _ = p.getBasePositionAndOrientation(cubeId)
        ballPos, _ = p.getBasePositionAndOrientation(ballId)

        dx = ballPos[0] - cubePos[0]
        dy = ballPos[1] - cubePos[1]

        lift_factor = 1.0

        force_magnitude = 500
        final_force = [
            dx * force_magnitude,
            dy * force_magnitude,
            lift_factor * force_magnitude
        ]

        print(f'Final force: {final_force}')
        print(f'x: {dx}, y: {dy}')
        
        p.applyExternalForce(objectUniqueId=ballId, 
                             linkIndex=-1, 
                             forceObj=final_force, 
                             posObj=ballPos, 
                             flags=p.WORLD_FRAME)
        
        cooldown = 50 

    # Decrease cooldown
    if cooldown > 0:
        cooldown -= 1

    time.sleep(1./240.)

p.disconnect()