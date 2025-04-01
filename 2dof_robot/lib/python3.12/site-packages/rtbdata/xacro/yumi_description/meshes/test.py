import swift
env = swift.Swift()
env.launch(realtime=True) # browser="chrome")

import spatialgeometry as sg
import sys

x = sg.Mesh(sys.argv[1])

env.add(x)
env.step()
env.hold()