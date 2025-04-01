import numpy as np
from roboticstoolbox import ET, Link, Robot
from spatialmath import quaternion
from math import pi

if __name__ == '__main__':
    a0 : float = 1.0
    d1 : float = 1.0
    d2 : float = 1.0
    d3 : float = 1.0
    
    q1 : float = 0.0
    q2 : float = 0.0
    q3 : float = 0.0
    q : np.ndarray = np.array([q1, q2, q3])
    
    A01 : ET = ET.tx(a0) * ET.Rz()
    A12 : ET = ET.tx(d1) * ET.Rx(pi/2) * ET.Rz()
    A23 : ET = ET.tx(d2) * ET.Rz()
    A3f : ET = ET.tx(d3) * ET.Rx(-pi/2)
    
    link0: Link = Link(A01,
                      name='link0',
                      qlim=[-pi,pi],
                      parent=None,
                      jindex=0,
                      joint='R')
    
    link1: Link = Link(A12,
                      name='link1',
                      qlim=[-pi,pi],
                      parent=link0,
                      jindex=1,
                      joint='R')
    
    link2: Link = Link(A23,
                      name='link2',
                      qlim=[-pi,pi],
                      parent=link1,
                      jindex=2,
                      joint='R')
    
    ef : Link = Link(A3f,
                     name='ef',
                     parent=link2)
    
    links: Link = [link0, link1, link2, ef]
    
    robot: Robot = Robot(links, name='3dof')
    print(Robot)
    
    q[1] = -pi/2
    
    A = robot.fkine(q)
    print(A)
    
    phi, theta, psi = A.eul()
    print(f'phi = {phi}, theta = {theta}, psi = {psi}')
    
    quat: quaternion.UnitQuaternion = A.UnitQuaternion()
    print(f'quat = {quat}')
    
    sol: tuple = robot.ik_NR(A)
    print(f'q = {np.round(sol[0], 4)}')
    
    robot.plot(q, block=True, backend='pyplot')