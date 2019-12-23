# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import dfgeom
from dfgeom import *
import gen
import math
from math import sqrt

#return p.addInstruction("float", "max", [b,a], ssaid=self.name+"_result")

# float a=atan(pos.x, pos.y);
# vec3 dd = vec3(sin(3*a+0.3*length(pos)));
# vec3 color = vec3(dd.x, 1, 1);
	

def glslFunction():
    p=gen.Program()
    
    inp = p.addConstant("Vec3", [0,0,0], ssaid="pos")
    r = shape.toGraph(p, inp)
    v = p.addConstant("Vec3", [1.0,1.0,1.0])
    out = p.addInstruction("Vec4", "vec4",[r, v])
    p.addInstruction("", "return", [out], ssaid="return")
    p = gen.glslDump(p)
    return (p[0], p[1]) #str(p[1])+"\n return "+out.ssaid+";")

def evalField(x,y,z):
        return dfgeom.evalField(shape, x,y,z)

### OTHER STUFF


#### Shape description
u = Sphere(radius=0.0)
for i in range(3):
        b = Rotate(Translate(Box([6,3.0,0.5]), [5,0,0]), x=2*i) 
        u = Union(u, b)
c=Rotate(Cylinder([1.0,1.0,0.0]),z=3.14/2)
u = Difference(u, c)        
                 
shape = u


