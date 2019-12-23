# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import dfgeom
from dfgeom import *

def evalField(x,y,z):
    return dfgeom.evalField(shape, x,y,z)

#### Shape description
b = Box([1.5,0.5,1.0])
u = Sphere([0.0,0.0,0.0], 0.9)
shape = Difference(b, u)
