import numpy as np
from roboticstoolbox import ET, Link, Robot
from spatialmath import quaternion
from math import pi

if __name__ == '__main__':
    d1 : float = 1.0
    d2 : float = 1.0
    d3 : float = 1.0
    d4 : float = 0.5
    
    q0 : float = 0.0
    q1 : float = 0.0
    q : np.ndarray = np.array([q0, q1])
    
    A01 : ET = ET.tz(0)
    A12 : ET = ET.tz(5)
    A23 : ET = ET.tz(0)
    A34 : ET = ET.tz(10) * ET.Rz()
    
    A45 : ET = ET.tx(d1) * ET.Rx(pi/2) * ET.Rz()
    A56 : ET = ET.tx(d2) * ET.Rz()
    A67 : ET = ET.tx(d3) * ET.Rx(-pi/2) * ET.Rz()
    A7f : ET = ET.tx(d4) * ET.Ry(pi/2)
    
    link0: Link = Link(A01,
                      name='link0',
                      qlim=[-pi,pi],
                      parent=None,
                      jindex=0,
                      joint='P')
    
    link1: Link = Link(A12,
                      name='link1',
                      qlim=[-pi,pi],
                      parent=link0,
                      jindex=1,
                      joint='P')

    link2: Link = Link(A12,
                    name='link2',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')
    
    link3: Link = Link(A12,
                    name='link3',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')
    
    link4: Link = Link(A12,
                    name='link4',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')
    
    link5: Link = Link(A12,
                    name='link5',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')

    link6: Link = Link(A12,
                    name='link6',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')

    link7: Link = Link(A12,
                    name='link7',
                    qlim=[-pi,pi],
                    parent=link0,
                    jindex=1,
                    joint='P')
    
    ef : Link = Link(A7f,
                     name='ef',
                     parent=link7)
    
    links: Link = [link0, link1, ef]
    
    robot: Robot = Robot(links, name='2dof')
    print(Robot)
    
    q[1] = pi/2
    A = robot.fkine(q)
    print(A)
    
    phi, theta, psi = A.eul()
    print(f'phi = {phi}, theta = {theta}, psi = {psi}')
    
    quat: quaternion.UnitQuaternion = A.UnitQuaternion()
    print(f'quat = {quat}')
    
    sol: tuple = robot.ik_NR(A)
    print(f'q = {np.round(sol[0], 4)}')
    
    robot.plot(q, block=True, backend='pyplot')