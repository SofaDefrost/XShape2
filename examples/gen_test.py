from gen import *
from pprint import *

class Twist(object):
        def __init__(self, target, angle):
                self.target=target
                self.angle=angle
        
        def toGraph(self, p, input):
                a=p.addConstant("float", self.angle)
                t = p.addInstruction("Vec3", "sdTwist", [a, p.nodes["pos"]])
                return self.target.toGraph(p, t)

class Transform(object):
        def __init__(self, target, translate=[0,0,0], rotate=[0,0,0], scale=[1,1,1]):
                self.matrix = "matrix[]"
                self.target = target

        def toGraph(self, p, input):
                m=p.addConstant("mat4", self.matrix)
                q = p.addInstruction("Vec3", "invert", [m])
                t = p.addInstruction("Vec3", "mul", [q,input])
                return self.target.toGraph(p, t)
                
        def toSelection(self, p, input, path):
                if len(path) == 1 and self == path[0]:
                        return self.toGraph(p, input)
                return self.target.toSelection(p, input, path[1:])
                
                
class Union(object):
        def __init__(self, a, b):
                self.a = a
                self.b = b

        def toGraph(self,p,input):
                a = self.a.toGraph(p,input)
                b = self.b.toGraph(p,input)
                return p.addInstruction("float", "min", [a,b])

        def toSelection(self, p, input, path):
                if len(path) == 1 and self == path[0]:
                        return self.toGraph(p, input)
                raise Exception("Invalid path query "+str(self)+" vs "+str(path) )                
                  

class Difference(object):
        def __init__(self, a, b):
                self.a = a
                self.b = b

        def toGraph(self,p,input):
                a = self.a.toGraph(p,input)
                b = self.b.toGraph(p,input)
                r = p.addInstruction("float", "-", [a]) 
                return p.addInstruction("float", "max", [r,b])

        def toSelection(self, p, input, path):
                if len(path) == 1 and self == path[0]:
                        return self.toGraph(p, input)
                elif self.a == path[1]:
                        return self.a.toSelection(p, input, path[1:])
                elif self.b == path[1]:
                        return self.b.toSelection(p, input, path[1:])
                raise Exception("Invalid path query "+str(self)+" vs "+str(path) )                

class Intersection(object):
        def __init__(self, a, b):
                self.a = a
                self.b = b

        def toGraph(self,p,input):
                a = self.a.toGraph(p, input)
                b = self.b.toGraph(p, input)
                return p.addInstruction("float", "max", [a,b])
                
                
class Sphere(object):
        def __init__(self, center, radius):
                self.center=center
                self.radius=radius

        def toGraph(self, p, input):
                r = p.addConstant("float", self.radius)
                c = p.addConstant("Vec3", self.center)
                return p.addInstruction("float", "sdSphere", [c,r,input])
        
        def toSelection(self, p, input, path):
                if len(path) == 1 and self == path[0]:
                        return self.toGraph(p, input)
                raise Exception("Invalid path query "+str(self)+" vs "+str(path) )                


def testGen():
        s1 = Sphere([1,2,3],3)
        t1 = Transform(s1, [2,2,2])
        s2 = Sphere([1,2,3],4)
        u1  = Union(s1, s2)
        t  = Twist(u1, 10)
        t  = Twist(t, 20)
        gg  = Intersection(t,u1)

        p=Program()
        p.addConstant("Vec3", [0,0,0], ssaid="pos")
        gg.toGraph(p, p.nodes["pos"])
        glslDump(p)
                        
   
def testSelection():
        s1 = Sphere([1,2,3],3)
        t1 = Transform(s1, [2,2,2])
        s2 = Sphere([0,0,0],1)
        u = Difference(t1,s2)

        p=Program()
        p.addConstant("Vec3", [0,0,0], ssaid="pos")
        u.toSelection(p, p.nodes["pos"], [u,t1,s1])
        pprint(glslDump(p))

     
testSelection()
