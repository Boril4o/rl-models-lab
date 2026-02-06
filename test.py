import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")
ballId = p.loadURDF("sphere2.urdf", [0, 0, 0.5])
cubeId = p.loadURDF("cube.urdf", [2, 0, 0.5])

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