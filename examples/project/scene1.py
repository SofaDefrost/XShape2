""" type: SofaContent """
import sys
import os
import Sofa
import Sofa.Core
from Sofa.constants import Key

import pyximport; pyximport.install(reload_support=True)
import dfgeom
import gen


def bindShapeToShader(node):
    type = node.getData("xtype").value
    if type == None:
        for child in node.children:
            cc = toPythonShape(child, kmap)
            if cc != None:
                return cc
        return None


def toPythonShape(node, kmap):
    type = node.getData("xtype").value
    if type == None and hasattr(node, "children"):
        for child in node.children:
            cc = toPythonShape(child, kmap)
            if cc != None:
                return cc
        return None

    if type == "XShape":
        c = list(node.children)
        if len(c) == 0:
            return None
        u=toPythonShape(c[0], kmap)
        for child in c[1:]:
                uc = toPythonShape(child, kmap)
                if uc != None:
                    u=dfgeom.Union(u, uc)
        return u
    elif type == "Difference":
        c = list(node.children)
        if len(c) == 0:
            return None
        left=toPythonShape(c[0], kmap)
        right=toPythonShape(c[1], kmap)

        d = dfgeom.Difference(left, right)
        d.setName(node.name.value)

        t = dfgeom.Translate(d, node.translation.value)
        t.setName(node.name.value)
        return t
    elif type == "Intersection":
        c = list(node.children)
        if len(c) == 0:
            return None
        left=toPythonShape(c[0], kmap)
        right=toPythonShape(c[1], kmap)
        d = dfgeom.Intersection(left, right)
        d.setName(node.name.value)
        return d
    elif type == "Box":
        b = dfgeom.Box(list(node.size.value))
        b.setName(node.name.value)
        def update():
                b.setAttr("size", node.size.value)                
        #node.componentState.onChange = update
        return b
    elif type == "Sphere":
        s = dfgeom.Sphere(position=list(node.position.value), radius=node.radius.value)
        s.setName(node.name.value)
        def update():
                s.setAttr("position", list(node.position.value))
                s.setAttr("radius", node.radius.value)
        #node.componentState.onChange = update
        return s
    elif type == "Twist":
        ps = toPythonShape(node.children[0], kmap)
        s = dfgeom.Twist(ps, angle=node.angle.value)
        s.setName(node.name.value)
        return s
    elif type == "Offset":
        ps = toPythonShape(node.children[0], kmap)
        s = dfgeom.Offset(ps, node.offset.value)
        s.setName(node.name.value)
        return s
    elif type == "Tiling":
        ps = toPythonShape(node.children[0], kmap)
        s = dfgeom.Tiling(ps, factor=node.mod.value)
        s.setName(node.name.value)
        return s    
    print("Un-recognized type:"+str(type))
    return None

class Center(Sofa.Core.Controller):
        def __init__(self):
                Sofa.Core.Controller.__init__(self)
                
        def draw(self, vparams):
                pass

class Box(Sofa.Prefab):
        def __init__(self, size=[0,0,0], **kwargs):
                Sofa.Prefab.__init__(self, **kwargs)
                self.addData(name="size", value=size, type="Vec3d", group="XShape's properties", help="size in X,Y,Z")
                self.addData(name="xtype", value="Box", type="string", group="Infos", help="Type of the object")
                
                self.addData(name="points", value=[[0.0,0.0,0.0],[1.0,1.0,1.0]], type="vector<Vec3d>", group="XShape's outputs", help="")
                self.xtype.setReadOnly(True)
                #self.addObject(Center())
                def update():
                        x,y,z = self.size.value 
                        self.points.value=[[-x,-y,-z], [x,y,z]]
                        print("HELLO WORLD ", self.points.value)
                
                self.size.connectX(self.points, update)
                #self.size.onChanged = lUpdate

        def onDraw(self, drawTools):
            drawTools.drawPoint()

class Twist(Sofa.Prefab):
        def __init__(self, child=None, angle=0, **kwargs):
                Sofa.Prefab.__init__(self, **kwargs)
                self.addData(name="angle", value=angle, type="double", group="XShape's properties", help="Angle of the twisting")
                self.addData(name="xtype", value="Twist", type="string", group="Infos", help="Type of the object")
                self.xtype.setReadOnly(True)

                self.addChild(child)

                def update():
                        pass
                
                self.angle.onChanged = update
                child.componentState.connectX(self.componentState, update)

class Offset(Sofa.Prefab):
        def __init__(self, child=None, offset=0, **kwargs):
                Sofa.Prefab.__init__(self, **kwargs)
                self.addData(name="offset", value=offset, type="double", group="XShape's properties", help="Amount of offset")
                self.addData(name="xtype", value="Offset", type="string", group="Infos", help="Type of the object")
                self.xtype.setReadOnly(True)

                self.addChild(child)
                def update():
                        pass
                self.offset.onChanged = update
                child.componentState.connectX(self.componentState, update)


class Tiling(Sofa.Prefab):
        def __init__(self, child=None, space=[0,0,0], **kwargs):
                Sofa.Prefab.__init__(self, **kwargs)
                self.addData(name="mod", value=list(space), type="Vec3d", group="XShape's properties", help="Angle of the twisting")
                self.addData(name="xtype", value="Tiling", type="string", group="Infos", help="Type of the object")
                self.xtype.setReadOnly(True)

                self.addChild(child)

                def update():
                        pass
                
                self.mod.onChanged = update
                child.componentState.connectX(self.componentState, update)                

def Sphere(name="Sphere", position=[0.0,0.0,0.0], radius=1.0):
        self = Sofa.Core.Node(name)
        self.addData(name="position", value=position, type="Vec3d", help="The position of the center of the Sphere", group="XShape's properties")
        self.addData(name="radius", value=radius, type="double", help="Sphere's radius", group="XShape's properties")
        self.addData(name="xtype", value="Sphere", type="string", group="Infos", help="Type of the object")
        self.xtype.setReadOnly(True)
        
        def update():
                pass
                #print("Updating self state...because internal state changed => ")
                        
        self.position.onChanged = update
        self.radius.onChanged = update                   
        return self

def Intersection(name="Intersection", left=None, right=None):
        self = Sofa.Core.Node(name)
        self.addData(name="xtype", value="Intersection", type="string", group="Infos", help="Type of the object")
        if left is not None:
                self.addChild(left)
                if right is not None:
                        self.addChild(right)
        self.xtype.setReadOnly(True)

        def update():
                pass

        left.componentState.connectX(self.componentState, update)
        right.componentState.connectX(self.componentState, update)

        return self

def Difference(name="Difference", left=None, right=None):
        self = Sofa.Core.Node(name)
        self.addData(name="translation", value=[0.0,0.0,0.0], type="Vec3d", group="XShape's properties", help="Type of the object")
        self.addData(name="xtype", value="Difference", type="string", group="Infos", help="Type of the object")

        if left is not None:
                self.addChild(left)
                if right is not None:
                        self.addChild(right)
        self.xtype.setReadOnly(True)
        def update():
            pass
        self.translation.onChanged = update

        def update():
                pass

        left.componentState.connectX(self.componentState, update)
        right.componentState.connectX(self.componentState, update)

        return self

class XShape(Sofa.Core.RawPrefab):
        def __init__(self, *args, **kwargs):
                Sofa.Core.RawPrefab.__init__(self, *args, **kwargs)
                self.addData(name="xtype", value="XShape", type="string", group="Infos", help="Type of the object")
                self.xtype.setReadOnly(True)

        def addShape(self, node, **kwargs):
                self.componentState.value = "Loading"
                c = self.add(node, **kwargs)

                def update():
                        self.componentState.value = "Valid"

                c.componentState.connectX(self.componentState, update)
                return node

        def getShapeFunction(self):
                kmap = {}
                print("GenerateGeometricalTree")
                shape=toPythonShape(self, kmap)
                return (shape, kmap)

class XShapeController(Sofa.Core.Controller):
        def __init__(self, *args, **kwargs):
                Sofa.Core.Controller.__init__(self, *args, **kwargs)
                self.xshape = kwargs.get("xshape", None)
                self.field = kwargs.get("field", None)
                self.map = {}

                def update():
                        shape, self.map = self.xshape.getShapeFunction()
                        def evalF(x,y,z):
                                #return -1.0
                                return dfgeom.evalField(shape,x,y,z)
                        self.field.pyEvalFunction.value = evalF

                        def glslF():
                            dfgeom.resetIdx()
                            #print("")
                            #print("================================================")
                            #print("idx: "+str(dfgeom.getIdx()))
                            p=gen.Program()

                            inp = p.addConstant("Vec3", [0,0,0], ssaid="pos")
                            r = shape.toGraph(p, inp)
                            v = p.addConstant("Vec3", [1.0,1.0,1.0])
                            out = p.addInstruction("Vec4", "vec4",[r, v])

                            p = gen.glslDump(p)
                            #print("code" + str(p))
                            dfgeom.idx = 0
                            return (p[0], gen.toStr(p[1])+"\n return "+out.ssaid+";")
                        self.field.pyGlslFunction.value = glslF

                self.xshape.componentState.connectX(self.field.pyEvalFunction, update)
 
        def onIdle(self, args):
                if len(self.map) != 0:
                        for k in self.map:
                                print("BIND DATA ", k)
             
def connectData(geometry, shader):
        if hasattr(geometry, "children"):        
                for child in geometry.children:
                        connectData(child, shader)
        
        if hasattr(geometry, "objects"):                        
                for object in geometry.objects:             
                        connectData(object, shader)

        
        for data in geometry.__data__:       
                d = geometry.name.value + "_" + data
                sd = shader.getData(d)
                if sd is not None and not sd.hasParent():
                        #sd.setParent(geometry.getData(data)) 
                        sd.value = geometry.getData(data).value
                        
def createScene(root):
        s = root.addChild("Settings")
        s.add("RequiredPlugin", name="SofaImplicitField3")

        c = root.addChild("Demo")
        geometry = c.add(XShape, name="XShape")
        ss = Twist(name="Twist1", child=Box(name="Box1", size=[0.5,1.0,1.0]))
        d = Difference(name="Difference0", left=ss, right=Sphere(name="Sphere2"))
        s = geometry.addShape(Offset(d, offset=0.5))

        #s.addData(name="translate", value=[0.0,0.0,0.0], type="Vec3d", group="XShape's properties", help="size in X,Y,Z")

        #def update():
        #    s.Twist.angle.value = s.translation.value[1]
        #s.translation.connectX(s.Twist.angle, update)

        #geometry.addShape(Difference(name="Difference1", left=Sphere(name="Sphere3"), right=Sphere(name="Sphere4")))
        #geometry.addShape(Intersection(name="Intersection", left=Sphere(name="Sphere5"), right=Sphere(name="Sphere6")))

        #geometry.addShape(Intersection(name="Intersection2", left=Box(name="Box2", size=[1.0,1.0,1.0]), right=Tiling(name="Tiling", child=Sphere(name="Sphere7"), space=[0.1,0.1,0.1])))

        field = c.addObject("CustomField")
        visuals = c.addChild("Visuals")
        pc = visuals.addChild("PointCloud")
        pc.add("VisualTransform", transform=[-3.0,0,0,0,0,0,1])
        pc.add("PointCloudImplicitFieldVisualization", field=field.getLinkPath(), evalFunction=field.evalFunction.getLinkPath(), box=[-1,-1,-1,1.0,1.0,1.0])
        
        sm = visuals.addChild("SurfaceMesh")
        sm.add("VisualTransform", transform=[3.,0,0,0,0,0,1])
        sm.add("SurfaceMeshGenerationFromScalarField", evalFunction=field.evalFunction.getLinkPath())

        sm = visuals.addChild("ShaderBaseRendering")
        shader = sm.add("ImplicitFieldShaderVisualization", field=field.getLinkPath())

        c.add(XShapeController, name="controller", xshape=geometry, field=field)
        
        def rebind():
                connectData(geometry, shader)
                
        geometry.componentState.onChange = rebind
