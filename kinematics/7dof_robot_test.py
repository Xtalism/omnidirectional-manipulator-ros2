import numpy as np
from roboticstoolbox import ET, Link, Robot
from spatialmath import quaternion
from math import pi

if __name__ == '__main__':
    d1: float = 1.0
    d2: float = 1.0
    d3: float = 1.0
    d4: float = 0.5
    
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    q4: float = 0.0
    q5: float = 0.0
    q6: float = 0.0
    
    p0: float = 5.0
    p1: float = 0.0
    p2: float = 10.0
    p3: float = pi
    p4: float = (-pi/2)
    p5: float = (pi/2)
    p6: float = 0.0
    
    r0: float = 0.0
    r1: float = 0.0
    r2: float = -5.0
    r3: float = 0.0
    r4: float = 0.0
    r5: float = (-pi/2)
    r6: float = pi

    q: np.ndarray = np.array([q0, q1, q2, q3, q4, q5, q6])
    p: np.ndarray = np.array([p0, p1, p2, p3, p4, p5, p6])
    r: np.ndarray = np.array([r0, r1, r2, r3, r4, r5, r6])

    A01: ET = ET.Ry(pi/2) * ET.tz()
    A12: ET = ET.Ry(-pi/2) * ET.tz()
    A23: ET = ET.Ry(pi/2)*ET.tz()
    A34: ET = ET.Rz()
    A45: ET = ET.tx(d1) * ET.Rx(pi/2) * ET.Rz()
    A56: ET = ET.tx(d2) * ET.Rz()
    A67: ET = ET.tx(d3) * ET.Ry(pi/2) * ET.Rz()
    A7f: ET = ET.tz(d4) * ET.Ry(-pi/2) * ET.Rx(-pi/2)

    link0: Link = Link(A01, name='link0', qlim=[-pi, pi], parent=None, jindex=0, joint='P')
    link1: Link = Link(A12, name='link1', qlim=[-pi, pi], parent=link0, jindex=1, joint='P')
    link2: Link = Link(A23, name='link2', qlim=[-pi, pi], parent=link1, jindex=2, joint='P')
    link3: Link = Link(A34, name='link3', qlim=[-pi, pi], parent=link2, jindex=3, joint='R')
    link4: Link = Link(A45, name='link4', qlim=[-pi, pi], parent=link3, jindex=4, joint='R')
    link5: Link = Link(A56, name='link5', qlim=[-pi, pi], parent=link4, jindex=5, joint='R')
    link6: Link = Link(A67, name='link6', qlim=[-pi, pi], parent=link5, jindex=6, joint='R')
    ef: Link = Link(A7f, name='ef', parent=link6)

    links: Link = [link0, link1, link2, link3, link4, link5, link6, ef]
    robot: Robot = Robot(links, name='7dof')
    
    # Imprimir enlaces del robot
    print(f"Robot links: {robot.links}")

    # Imprimir configuraciones q, p y r
    print(f"Joint configuration q: {q}")
    print(f"Joint configuration p: {p}")
    print(f"Joint configuration r: {r}")

    # Imprimir nombre del robot y número de enlaces
    print(f"Robot name: {robot.name}")
    print(f"Number of links: {len(robot.links)}")

    # Obtener la cinemática directa (FK) y mostrar resultados
    A = robot.fkine(q)
    print(f"Transformation A: {A}")
    b = robot.fkine(p)
    print(f"Transformation b: {b}")
    c = robot.fkine(r)
    print(f"Transformation c: {c}")

    # Imprimir ángulos de Euler
    phi, theta, psi = A.eul()
    print(f"Euler angles for A: phi = {phi}, theta = {theta}, psi = {psi}")
    
    phi, theta, psi = b.eul()
    print(f"Euler angles for b: phi = {phi}, theta = {theta}, psi = {psi}")
    
    phi, theta, psi = c.eul()
    print(f"Euler angles for c: phi = {phi}, theta = {theta}, psi = {psi}")

    # Convertir a cuaterniones y mostrar
    quat: quaternion.UnitQuaternion = A.UnitQuaternion()
    print(f"Quat for A: {quat}")
    
    quat: quaternion.UnitQuaternion = b.UnitQuaternion()
    print(f"Quat for b: {quat}")
    
    quat: quaternion.UnitQuaternion = c.UnitQuaternion()
    print(f"Quat for c: {quat}")

    # Resolver la cinemática inversa (IK)
    sol: tuple = robot.ik_NR(A)
    print(f"IK solution for A (q): {np.round(sol[0], 3)}")
    
    sol: tuple = robot.ik_NR(b)
    print(f"IK solution for b (p): {np.round(sol[0], 3)}")
    
    sol: tuple = robot.ik_NR(c)
    print(f"IK solution for c (r): {np.round(sol[0], 3)}")

    # Graficar las configuraciones
    robot.plot(q, block=True, backend='pyplot')
    robot.plot(p, block=True, backend='pyplot')
    robot.plot(r, block=True, backend='pyplot')