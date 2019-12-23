# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import dfgeom
from dfgeom import *
import gen
import math
from math import sqrt


#### Shape description
b = Box([1.5,0.5,1.0])
shape = b

u = Sphere([0.0,0.0,0.0], 0.9)
shape1 = shape = Union(b, u)

for i in range(10):
        u2 = Sphere([1.0,0.0,0.0], 0.4)
        u3 = Translate(u2, [-3+0.5*i,0,0.8])
        shape = Difference(shape, u3)        

#shape = shape1 #Difference(shape, u3)

#b = Box([5.0,0.3,0.3])
#shape = Union(shape, b)

def evalField(x,y,z):
    return dfgeom.evalField(shape, x,y,z)
    
    
def glslFunction():
    p=gen.Program()
    inp = p.addConstant("Vec3", [0,0,0], ssaid="pos")
    r = shape.toGraph(p, inp)
    v = p.addConstant("Vec3", [1.0,1.0,1.0])
    out = p.addInstruction("Vec4", "vec4",[r, v])    
    p.addInstruction("", "return", [out], ssaid="return")
    p = gen.glslDump(p)
    return (p[0], p[1])

