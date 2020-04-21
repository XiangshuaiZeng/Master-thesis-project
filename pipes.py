import Part, Draft, Mesh
from FreeCAD import Base
from math import *


def pipe_line(L, D, r, theta):
	base_curve = []
	base_curve.append(FreeCAD.Vector(r - D, -L))
	for delta in range(180 - theta+1):
		base_curve.append(FreeCAD.Vector(r*cos(delta/180.0*pi)-D, r*sin(delta/180.0*pi)-D/tan(theta/2.0/180*pi)))
	
	base_curve.append(FreeCAD.Vector(r-(L+r/tan(theta/2.0/180*pi))*sin(theta/180.0*pi) + D*cos(theta/180.0*pi), r/tan(theta/2.0/180*pi)-(L+r/tan(theta/2.0/180*pi))*cos(theta/180.0*pi)-D*sin(theta/180.0*pi)))
	
	pipe_curve = Part.makePolygon(base_curve)
	return pipe_curve

def get_pipe(L, D, r, theta, thick, h):
	curve_up1 = pipe_line(L, 0, r, theta)
	curve_up2 = pipe_line(L, thick, r, theta)
	
	curve_down1 = pipe_line(L, D, r, theta)
	curve_down2 = pipe_line(L, D + thick, r, theta)
	
	# make surfaces
	curve_up = Part.makeRuledSurface(curve_up1, curve_up2)
	curve_down = Part.makeRuledSurface(curve_down1, curve_down2)
	
	pipe_up = curve_up.extrude(Base.Vector(0,0,h))
	pipe_down = curve_down.extrude(Base.Vector(0,0,h))
	
	pipe = Part.makeCompound([pipe_up, pipe_down])
	return pipe

L = 500 
thick = 10
r = 40
h = 80  # height 

theta = 90  # the angle should only be integer
D = 145
for theta in range(80, 101, 4):
	for D in range(135, 151, 3):
		pipe = get_pipe(L, D, r, theta, thick, h)
		# export stl
		doc = App.getDocument('pipes')
		pf = doc.addObject("Part::Feature","MyShape")
		pf.Shape = pipe
		address = '/home/joshua/robot_arm_ws/src/mrm_description/meshes/pipe_mesh/d%s_%s.stl' % (D, theta)
		Mesh.export([pf], address)
