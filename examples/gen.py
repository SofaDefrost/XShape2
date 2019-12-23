
def toStr(s):
        t = ""
        for i in s:
                t+=i+"\n"
        return t

class Node(object):
        def __init__(self, ssaType, ssaid, value, type, params):
                self.ssaid = ssaid
                self.ssaType = ssaType
                self.value = value
                self.params = params
                self.type = type

        def isConstant():
                return value == ""

class Constant(object):
        def __init__(self, value):
                self.value=value
                self.type = "constant"
                
class Program(object):
        def __init__(self):
                self.nodes = {} 
                self.ssaid = 0

        def addConstant(self, ssaType, value, ssaid=None):
                if ssaid == None:
                        self.ssaid = self.ssaid+1
                        ssaid = "ssa"+str(self.ssaid)
                if isinstance(value, list):
                        #valueO=[]
                        #for i in range(len(value)):
                                #valueO.append(Constant(value[i]))
                        value = list(map( lambda i: Constant(value[i]), list(range(len(value))) ))
                        
                else:
                        value = [Constant(value)]        
                self.nodes[ssaid] = Node(ssaType, ssaid, ssaType, "constant", value) 
                        
                return self.nodes[ssaid]
                
        def addInstruction(self, ssaType, ssaIr, params, ssaid=None):
                if ssaid == None:
                        self.ssaid = self.ssaid+1
                        ssaid = "ssa"+str(self.ssaid)
                self.nodes[ssaid] = Node(ssaType, ssaid, ssaIr, "op", params) 
                return self.nodes[ssaid]

def topologicalSort(nodes):
        tmp = []
        c = {}
        for k, v in nodes.items():
            tmp.append(v)
            c[k] = v
        
        variables=[]
        ordered=[]
        while len(c) != 0:
                #print("QUEUE: "+str(len(tmp)))
                for candidate in list(c.values()):
                        #print("\nTRYING: "+str(candidate.value))
                        allDependenciesKnown=True
                        
                        if candidate.type != "constant":
                                for p in candidate.params:
                                        #print("CHECKING: " + str(p.ssaid))
                                        if p.ssaid in c:
                                                #print("BREAKS")
                                                allDependenciesKnown=False
                                                break          
                                if allDependenciesKnown:
                                        #print("ACCEPTING: "+candidate.ssaid)
                                        ordered.append(candidate)
                                        del c[candidate.ssaid]  
                        else:
                                variables.append(candidate)
                                del c[candidate.ssaid]  
                                  
        return (variables, ordered)

def type2GLSL(p):
        t = {
                "float" : "float",
                "Vec3"  : "vec3",
                "Vec4"  : "vec4",
                "mat4" : "mat4",
                ""     : ""
        }
        return t[p]

def value2GLSL(p):
        t = ""
        for i in p:
                t+=str(i.value)+" "
        return t[0:-1]
        
def glslDump(p):
        v, i = topologicalSort(p.nodes)
        vglsl = []
        for a in v:       
             tmp = "uniform " + type2GLSL(a.ssaType) + " " +a.ssaid;  
             vglsl.append( (a.ssaid, type2GLSL(a.ssaType), tmp, value2GLSL(a.params)) ) 
                     
        iglsl = []
        for a in i:   
             if a.ssaid == "return":
                tmp = "return " + a.params[0].ssaid + ";"
                iglsl.append(tmp)
                continue
                 
             tmp = type2GLSL(a.ssaType) + " " + a.ssaid +" = " + a.value + "(" 
             for p in a.params:
                if isinstance(p, Constant):
                     tmp += str(p.value) + ", "
                else:
                     tmp += str(p.ssaid) + ", "                                     
             if tmp[-2:] == ", ":
                tmp = tmp[0:-2]
             tmp += ");"
             iglsl.append(tmp)
        
        return (vglsl, iglsl)             

