import pyximport; pyximport.install(reload_support=True)
import sys
import difigeometry
import dfgeom

import math
from math import sqrt

#b = dfgeom.Box([0.5,0.5,1.0])
#u = dfgeom.Sphere([0.0,0.0,0.0], 0.5)

#s = dfgeom.Sphere([0.0,0.0,0.0], 0.5)

#shape = dfgeom.Difference(
#                dfgeom.Difference(b, u),
#                dfgeom.Sphere([1.0,0.5,0.0], 1.0)
#                )

#shape = dfgeom.Difference(
#                dfgeom.Union(s, shape),
#                dfgeom.Box([1.0,0.1,1.2])
#                )

#shape = dfgeom.Twist(shape, [1,1.0,1.0])

#def glslFunction():
#    context = {}
#    function = shape.toGLSL(context)
#    return (context.values(), function);

#def evalField(x,y,z):
#    return dfgeom.evalField(shape, x,y,z)
