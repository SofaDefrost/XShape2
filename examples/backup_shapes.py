""" type: SofaContent """
import sys
import os
import Sofa
import Sofa.Core
from Sofa.constants import Key

import pyximport; pyximport.install(reload_support=True)
import dfgeom

def toPythonShape(node):
    print("TO PYY: "+str(node.name))
    type = node.getData("type")
    if type == None:
        for child in node.getChildren():
            cc = toPythonShape(child)
            if cc != None:
                return cc
        return None

    if node.type == "shape.ShapeTree":
        c = node.getChildren()
        if len(c) == 0:
            return None
        u=toPythonShape(c[0])
        for child in c[1:]:
                uc = toPythonShape(child)
                if uc != None:
                    u=dfgeom.Union(u, uc)

        print("OUT: "+str(u))
        return u
    elif node.type == "shape.Difference":
        c = node.getChildren()
        if len(c) == 0:
            return None
        left=toPythonShape(c[0])
        right=toPythonShape(c[1])
        return dfgeom.Difference(left, right)
    elif node.type == "shape.Box":
        print("BOX ? "+str(node.size)+ "=================")
        b = dfgeom.Box(node.size[0])
        kmap[node.getLinkPath()]=(node, b)
        return b
    elif node.type == "shape.Sphere":
        print("SPHERE GEOM: "+str(node.center[0]))
        s = dfgeom.Sphere(center=node.center[0], radius=node.radius)
        kmap[node.getLinkPath()]=(node, s)
        return s
    print("Un-recognized type:"+str(node.type))
    return None

def Sphere(name="Sphere", position=[1.0,1.0,1.0], radius=0.0):
        self = Sofa.Core.Node(name)                
        self.addData(name="position", value=[1.0,1.0,1.0], type="Vec3d", help="The position of the center of the Sphere", group="XShape's properties")
        self.addData(name="radius", value=1.0, type="double", help="Sphere's radius", group="XShape's properties")
        self.addData(name="xtype", value="Sphere", type="string", group="Infos", help="Type of the object")
        self.xtype.setReadOnly(True)
        return self
        
def Difference(name="Difference", left=None, right=None):
        self = Sofa.Core.Node(name)                        
        self.addData(name="xtype", value="Difference", type="string", group="Infos", help="Type of the object")
        if left is not None:
                self.addChild(left)
                if right is not None:
                        self.addChild(right)
        self.xtype.setReadOnly(True)
        return self        
        
class XShape(Sofa.Core.RawPrefab):
        def __init__(self, *args, **kwargs):
                Sofa.Core.RawPrefab.__init__(self, *args, **kwargs)
                self.addData(name="xtype", value="XShape", type="string", group="Infos", help="Type of the object")
                self.xtype.setReadOnly(True)
                
        def getShapeFunction(self):
                print("GenerateGeometricalTree")
                b = dfgeom.Box([1.5,0.5,1.0])
                u = dfgeom.Sphere([0.0,0.0,0.0], 0.1)

                s = dfgeom.Sphere([0.0,0.0,0.0], 0.5)

                shape = dfgeom.Difference(
                        dfgeom.Difference(b, u),
                        dfgeom.Sphere([1.0,0.5,0.0], 1.0)
                        )

                shape = dfgeom.Difference(
                        dfgeom.Union(s, shape),
                        dfgeom.Box([1.0,0.1,1.2])
                        )
                return shape

class XShapeController(Sofa.Core.Controller):
        def __init__(self, *args, **kwargs):
                Sofa.Core.Controller.__init__(self, *args, **kwargs)
                self.xshape = kwargs.get("xshape", None)
                self.field = kwargs.get("field", None)
                
        def onKeypressedEvent(self, args):
                if args["key"] == Key.D:
                        shape = self.xshape.getShapeFunction()
                        def evalF(x,y,z):
                                return dfgeom.evalField(shape,x,y,z)                                
                        self.field.evalFunction.value = evalF
                                                                                   
def createScene(root):
        s = root.addChild("Settings")
        s.add("RequiredPlugin", name="SofaImplicitField3")        
        
        c = root.addChild("Demo")
        geometry = c.add(XShape, name="XShape")
        geometry.add(Sphere, name="Sphere1")
        geometry.add(Sphere, name="Sphere2")
        geometry.add(Difference(name="Difference", left=Sphere(name="Sphere3"), right=Sphere(name="Sphere4")))
        
        field = c.addObject("CustomField")
       
        c.add("PointCloudImplicitFieldVisualization", field=field.getLinkPath(), box=[-1,-1,-1,1.0,1.0,1.0])
        c.add(XShapeController, name="controller", xshape=geometry, field=field)

