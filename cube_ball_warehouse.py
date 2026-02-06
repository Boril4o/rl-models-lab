import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

ground = p.loadURDF("plane.urdf", [0, 0, 0])

playerCube = p.loadURDF("cube.urdf", [0, 0, 0.5])

ball = p.loadURDF("sphere2.urdf", [0, 5, 0.5])

p.changeDynamics(ball, -1, 0.1, restitution=0.5, lateralFriction=0.1)

p.changeDynamics(ground, -1, restitution=1)

p.changeDynamics(playerCube, -1, lateralFriction=0.1)

while True:
    p.applyExternalForce(playerCube, -1, [0, 5, 0], [0, 0, 0], p.LINK_FRAME)

    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()