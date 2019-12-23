import Sofa

import customshape
brin=None
class Updater(Sofa.PythonScriptController):
        def __init__(self, node, mapP):
                self.node=node
                self.smap=mapP
        
        def initGraph(self, node):
                self.node=node
        
       
        def onIdle(self):
                hasChanged = False
                for n,t in customshape.kmap.values():
                        for d in n.getDataFields():
                                if d in ["radius", "center"]:                               
                                       p = n.getData(d).value
                                       if type(p) == list:
                                         p = p[0]
                                       
                                       if t.getAttr(d) != p:
                                          print("CMP" +str(t.getAttr(d))+" " +str(p))
                                          t.setAttr(d, p)
                                          hasChanged = True
                
                if hasChanged:
                        self.brin = self.node.shape2.myField

                        self.brin.state = self.brin.state + 1                                        

