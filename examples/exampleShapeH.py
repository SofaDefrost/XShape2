# -*- coding: utf-8 -*-
import pyximport; pyximport.install(reload_support=True)
import sys
import dfgeom
import gen
import math

def dfTreeToSofa(dftree, node):
        dftree.toShapeTree(node)        


class Shape(object):
        def __init__(self, node, dftree):
                self.this = node.createChild("Shape") 
                c=self.this.createObject("CustomField", name="field", function="cubesphere.evalField", glslFunction="cubesphere.glslFunction")               
                shader = self.this.createObject("ImplicitFieldShaderVisualization", name="renderer", field="@field")
                #c.init()
                #shader.init()
                
                #Sync(self.this, self)
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
                                                
                for c in sofatree.children:
                        self.bindVariables(shader, c)
                        
def createScene(node):
        import cubesphere 
        #e = Editor(node)
        #Manipulator(e.tools)
        
        st = Shape(node, cubesphere)     
        
