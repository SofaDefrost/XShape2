# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import SofaPython
import dfgeom
import gen
import math
import Sofa
import SofaEditor
import SofaGeometry 
import OpenGL


OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLU import *

from splib.scenegraph import get
from math import sqrt

def dfTreeToSofa(dftree, node):
        dftree.toShapeTree(node)        

class Sync(Sofa.PythonScriptController):
        def __init__(self, node, shapetree):
                self.node = node
                self.name = "Sync"
                self.shapetree = shapetree
                self.oldState = self.shapetree.this.field.findData("state").value
                self.ssai_vars = []
                self.ssai = None
                self.selectedattrs = "radius"
                self.prevPos = None
                self.selectedData = None
                                
        def onMouseMove(self,x,y):
                if self.selectedData != None:
                        dx,dy = x- self.prevPos[0], y-self.prevPos[1] 
                        dx = dx*0.01
                        dy = dy*0.01
                        
                        if isinstance(self.selectedData.value, list):
                                nx,ny,nz = self.selectedData.value[0]
                                self.selectedData.value = [nx+dx,ny+dy,nz]
                                print("Mouse move "+str(self.selectedData.value))
                                self.prevPos=[x,y]
        
                        else:
                                self.selectedData.value += 0.01*dx
                                print("Mouse move "+str(self.selectedData.value))
                                self.prevPos=[x,y]
        
        
        def onIdle(self):
                if self.shapetree.this.field.findData("state").value != self.oldState:
                        print("UPDATE TREE "+ str(self.shapetree.tree))  
                        self.oldState = self.shapetree.this.field.findData("state").value 
                        #self.shapetree.registerShape()  
                        self.shapetree.bindVariables(self.shapetree.this.renderer, self.shapetree.tree)

class Manipulator(Sofa.PythonScriptController):
        def __init__(self, node):
                self.name = "Manipulators"
                self.listening = True      
                self.isActive = False
                self.selectedShape = None
                self.a =[0,0,0]
                self.b =[0,0,0]               
                
        def onMouseButtonLeft(self, x, y, state):
                if state == True:
                        if len(SofaEditor.getSelection()) > 0:
                                self.initMousePos = (x,y)
                                self.currMousePos = (x,y)
                                pathToSelection = SofaEditor.getSelection()[0]
                                selection = get(self.getContext().getRoot(), pathToSelection)
                                
                                # Check that the object is implementing the Shape protocole. 
                                if selection.getData("ssai"):
                                        self.selectedShape = selection
                                        print("DIR: "+ selection.name)
                                        print("Selection: " + str(SofaEditor.getSelection()))
                else:
                        self.selectedShape = None
                        
        def screenToWorldPoint(self, x, y, z ):
                if hasattr(self, "viewport"):
                        return gluUnProject(x,self.viewport[3]-y, z, self.mMatrix, self.pMatrix, self.viewport)
                return [0,0,0]
                                
        def onMouseMove(self, x, y):
                diffMousePos = (x-self.currMousePos[0], y-self.currMousePos[1])                
                self.currMousePos = (x,y)
                self.a = self.screenToWorldPoint(self.currMousePos[0],self.currMousePos[1], 0.05) 
                self.b = self.screenToWorldPoint(self.currMousePos[0],self.currMousePos[1], 1.00) 
                
                self.a
                                 
                #p = self.a                        
                #d = map(lambda i: self.b[i]-self.a[i], range(3))
                #s = math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
                #d = map(lambda i: d[i]/s, range(3))
                
                                      
        def draw(self):                
                        self.viewport = glGetIntegerv(GL_VIEWPORT)
                        self.pMatrix = glGetDoublev(GL_PROJECTION_MATRIX)
                        self.mMatrix = glGetDoublev(GL_MODELVIEW_MATRIX)

                #if self.selectedShape != None:
                        glDisable(GL_LIGHTING)
                        glBegin(GL_LINES)
                        glColor([1,0,0,1])
                        glVertex([0,0,0])
                        glVertex([2,0,0])

                        glColor([0,1,0,1])
                        glVertex([0,0,0])
                        glVertex([0,2,0])
                        
                        glColor([0,0,1,1])
                        glVertex([0,0,0])
                        glVertex([0,0,2])
                        
                        glColor([1,1,1,1])
                        glVertex(self.a)
                        glVertex(self.b)

                        glEnd()
                        glEnable(GL_LIGHTING)
                        
        
class EditorManager(Sofa.PythonScriptController):
        def __init__(self, node, tools):
                self.name = "SwitchTools"
                self.tools = tools
                self.selectedShape = None               


def Editor(parentNode):
        editor = parentNode.createChild("Editor")
        tools = editor.createChild("tools")
        EditorManager(editor, tools)
        
        return editor        

class Shape(object):
        def __init__(self, node, dftree):
                self.this = node.createChild("Shape") 
                c=self.this.createObject("CustomField", name="field", function="cubesphere.evalField", glslFunction="cubesphere.glslFunction")               
                shader = self.this.createObject("ImplicitFieldShaderVisualization", name="renderer", field="@field")
                #c.init()
                #shader.init()
                
                Sync(self.this, self)
                self.tree = self.this.createChild("ShapeTree")         
                self.dftree = dftree.shape
                self.registerShape()
                self.bindVariables(shader, self.this) 
                        
        def registerShape(self):
                self.dftree.toShapeTree(self.tree)            
                
        def bindVariables(self, shader, sofatree):
                g = sofatree.getData("ssai")
                if g != None:
                        n = g.value
                        v = sofatree.getData("ssai_vars").value.split(" ")
                        for i in v:
                                if len(i) != 0:
                                        t = shader.getData(n+"_"+i)
                                        if t != None:    
                                                print("BInD:" + n+"_"+i +" => "+str(t))
                                                t.setParent(sofatree.findData(i))
                                                
                for c in sofatree.getChildren():
                        self.bindVariables(shader, c)
                        
def createScene(node):
        import cubesphere 
        e = Editor(node)
        Manipulator(e.tools)
        
        st = Shape(node, cubesphere)     
        
