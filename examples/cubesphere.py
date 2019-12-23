# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import dfgeom
from dfgeom import *
import gen
import math
from math import sqrt

def toStr(s):
        t = ""
        for i in s:
                t+=i+"\n"
        return t

def glslFunction():
    p=gen.Program()
    
    inp = p.addConstant("Vec3", [0,0,0], ssaid="pos")
    r = shape.toGraph(p, inp)
    v = p.addConstant("Vec3", [1.0,1.0,1.0])
    out = p.addInstruction("Vec4", "vec4",[r, v])
    
    p = gen.glslDump(p)
    #print("code" + str(p))
    return (p[0], toStr(p[1])+"\n return "+out.ssaid+";")

def evalField(x,y,z):
    return evalField(shape, x,y,z)

### OTHER STUFF



#### Shape description
b = Box([1.5,0.5,1.0])
u = Sphere([0.0,0.0,0.0], 0.9)
shape = Union(b, u)
