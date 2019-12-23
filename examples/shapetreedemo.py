import pyximport; pyximport.install(reload_support=True)
import sys
#import SofaPython
#from SofaPython import Tools
#import difigeometry
import dfgeom
import gen
import math
from math import sqrt

b = dfgeom.Box([1.5,0.5,1.0])
u = dfgeom.Sphere([0.0,0.0,0.0], 0.7)

s = dfgeom.Sphere([0.0,0.0,0.0], 0.5)

shape = dfgeom.Difference(
                dfgeom.Difference(b, u),
                dfgeom.Sphere([1.0,0.5,0.0], 1.0)
                )

shape = dfgeom.Difference(
                dfgeom.Union(s, shape),
                dfgeom.Box([1.0,0.1,1.2])
                )

#shape = dfgeom.Scale(shape, [3.0,3.0,3.0])
#shape = dfgeom.Twist(shape, 1.0)
shape = dfgeom.Rotate(shape, x=3.14)
shape = dfgeom.Scale(shape, 1.0)
#shape = dfgeom.Twist(shape, 1.0)

#shape = dfgeom.Translate(shape, [0.0,0.0,0.0])
#shape = dfgeom.Tiling(shape, [1.0,1.0,1.0])
#shape = dfgeom.Intersection(dfgeom.Box([1.0,1.0,1.0]), shape)

def glslFunction_old():
    context = {}
    function = shape.toGLSL(context)
    return (context.values(), function);

def glslFunction():
    p=gen.Program()
    
    inp = p.addConstant("Vec3", [0,0,0], ssaid="pos")
    r = shape.toGraph(p, inp)
    v = p.addConstant("Vec3", [1.0,1.0,1.0])
    out = p.addInstruction("Vec4", "vec4",[r, v])
    
    p = gen.glslDump(p)
    #print("code" + str(p))
    return (p[0], p[1]+"\n return "+out.ssaid+";")

def evalField(x,y,z):
    return dfgeom.evalField(shape, x,y,z)
