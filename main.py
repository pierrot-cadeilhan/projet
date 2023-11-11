# -*- coding: utf-8 -*-
"""
Created on Sun Oct  8 13:59:06 2023

@author: Pierrot
"""

#--------------------[Préface]--------------------
#On acceptera l'existence de deux instances (ex: Node, Arc, ...) aux attributs
#identiques comme deux objets distincts.
#
#On considèrera que le chemin Path({n1, n2}, {}) est un chemin inexistant entre n1 et n2
#On pourra ainsi considérer des graphes d'arêtes de poids infinis
#On considèrera que le chemin Path({n0}, {}) est le chemin entre n0 et n0 sans arête
#On pourra ainsi considérer des graphes d'arêtes de coefficients nuls
#
#/!\ et des graphes d'arêtes à poids négatif ? Dijkstra ne marchera pas T^T
#
#En espérant que le code soit aussi clair pour vous qu'il ne l'est pour moi.
#--------------------[Fin de la préface]--------------------


#----------------------[Remarques]--------------------
#J'en suis à corriger les types et à gérer les conventions de notations
#méthodes et fonctions: Fonction()
#variables: maSuperVariable

#Path n'hérite pas de Tree car n'est pas forcément connexe


#--------------------[Fin des remarques]--------------------


import pyparsing as pp
import graphviz as gv

#OBJETS
class Node:
#---Dunder methods
    def __init__(self, name='Sommet'):
        self.name = str(name)
        
    def __repr__(self):
        """Description non ambigue de l'objet."""
        return f'Node("{self.name}")'
    
    def __str__(self):
        """Description claire de l'objet."""
        return self.name
    
    def __lt__(self, other):
        return self.name < other.name
    
#---Custom methods
    def ArcsTowards(self, graph)->set:
        """node.ArcsTowards(graph)
        Renvoie le sous-ensemble des arêtes de graph pointant vers node.
        """
        if not self in graph.nodes:
            print(f"Le sommet {self.name} n'est pas dans le graphe {graph.info['name']}.")
            return set()
        else:
            return set([a for a in graph.arcs if a.target == self])
    
    def ArcsFrom(self, graph)->set:
        """node.ArcsFrom(graph)
        Renvoie le sous-ensemble des arêtes de graph partant de node.
        """
        if not self in graph.nodes:
            print(f"Le sommet {self.name} n'est pas dans le graphe {graph.info['name']}.")
            return set()
        else:
            return set([a for a in graph.arcs if a.source == self])

class Arc:
#---Dunder methods
    def __init__(self, source, target, weight):
        self.source = source
        self.target = target
        self.weight = weight
        
    def __repr__(self):
        """Description non ambigue de l'objet."""
        return f'Arc({self.source}, {self.target}, {self.weight})'
        
    def __str__(self):
        """Description claire de l'objet."""
        return f'({self.source}, {self.target}, {self.weight})'

    def __lt__(self, other):
        return self.weight < other.weight
        
#---Custom methods
    def AsPath(self):
        """arc.AsPath()
        Renvoie le chemin équivalent à l'arc"""
        return Path(nodes={self.source, self.target}, start=self.source, arcs={self}, name='Arc')
    
class Graph:
#---Dunder methods
    def __init__(self, arcs, nodes, name='Graphe'):
        self.nodes = nodes
        self.arcs = arcs
        self.info = {'name':name}
    
    def __add__(self, other):
        """Somme de deux graphes comme la réunion
        des sommets et des arêtes."""
        g = self.__class__()
        g.info['name']=self.info['name'] + '+' + other.info['name']
        g.nodes = self.nodes | other.nodes
        g.arcs = self.arcs | other.arcs
        return g
    
    def __sub__(self, other):
        """Différence de deux graphes comme la réunion
        des sommets et la différence des arêtes."""
        g = Graph(set(), set())
        g.info['name']=self.info['name'] + '+' + other.info['name']
        g.nodes = self.nodes | other.nodes
        g.arcs = self.arcs - other.arcs
        return g
    
#---Custom methods
    def AddNode(self, node):
        """graph.AddNode(node)
        Ajoute node aux sommets de graph."""
        if node.name in [n.name for n in self.nodes]:
            print(f"Il y a déjà un sommet de nom {node.name} dans le graphe {self.info['name']}.")
        self.nodes.add(node)
        
    def RemoveNode(self, node):
        """graph.RemoveNode(node)
        Retire node des sommets de graph."""
        if not node in self.nodes:
            print(f"Le sommet {node.name} n'est pas dans le graphe {self.info['name']}.")
        else:
            self.nodes.remove(node)
        
    def AddArc(self, arc):
        """graph.AddArc(arc)
        Ajoute arc aux arcs de graph."""
        self.arcs.add(arc)
    
    def RemoveArc(self, arc):
        """graph.RemoveArc(arc)
        Retire arc des arcs de graph."""
        if not arc in self.arcs:
            print(f"L'arc {arc} n'est pas dans le graphe {self.info['name']}.")
        else:
             self.arcs.remove(arc)
        
    def GetNode(self, name)->Node:
        """graph.GetNode(name)
        Renvoie un sommet de graph dont le nom est name"""
        nodes = [n for n in self.nodes if n.name == str(name)]
        if nodes == []:
            print("Il n'y a pas de sommet du nom de {name} dans le graphe {self.info['name']}.")
            return None
        else:
            if len(nodes) > 1:
                print("Il y a plusieurs sommets du nom de {name} dans le graphe {self.info['name']}.")
            return nodes[0]
    
    def FromFile(self, parser, fileName:str):
        """graph.FromFile(parser, fileName)
        Charge dans graph le graphe représenté dans fileName selon le parseur parser"""
        
        #Parse les données selon la syntaxe de parser
        parsedData = parser.Parse(fileName)
        #Charge les parametres du graphe dans un dictionnaire
        parametres = {p[0].lower() :p[1] for p in parsedData[0]}
        
        nodes = parsedData[1]
        arcs = parsedData[2]
    
        self.info = parametres
        if not 'name' in parametres:
            self.info['name'] = 'Graphe'
    
        self.nodes = set([Node(n) for n in nodes])
        self.arcs = set([Arc(self.GetNode(a[0]), self.GetNode(a[1]), float(a[2])) for a in arcs])
    
    def Dijkstra(self, root):
        """graph.Dijkstra(root)
        Renvoie l'arbre de parenté des sommets de graph tel que:
            - sa racine est root
            - tout point d'une autre partie connexe de graph
            n'est pas dans cet arbre"""
       
        t = Tree(set(), set())
        
        #Tous les sommets dans toExplore | tree.nodes sont clés de dist
        toExplore = {root}
        dist = {root:0}
        parentArc = dict()
        
        while len(toExplore - t.nodes) > 0:
            #On prend le sommet déjà exploré le plus proche (il n'existera pas de plus court chemins que celui déjà trouvé)
            L = list(toExplore - t.nodes)
            nMin = L[0]
            dMin = dist[nMin]
            for node in L[1:]:
                if dist[node] <= dMin:
                    nMin = node
                    dMin = dist[node]
                    
            #On l'ajoute à l'arbre et on le retire de toExplore car il est entièrement déterminé
            t.AddNode(nMin)
            toExplore.remove(nMin)
            if nMin != root:
                #S'il ne sagit pas de la racine, on crée un arc entre lui et son parent
                t.AddArc(parentArc[nMin])
            
            #On parcourt ses sommets fils non encore déterminés, et on met à jour leurs propriétés
            for arc in nMin.ArcsFrom(self):
                if arc.target not in t.nodes:
                    dExplo = dist[nMin] + arc.weight
                    #Si ce sommet est déjà rencontré, on le met à jour
                    if arc.target in toExplore:
                        if dist[arc.target] > dExplo:
                            dist[arc.target] = dExplo
                            parentArc[arc.target] = arc
                    #Sinon on crée des données
                    else:
                        toExplore.add(arc.target)
                        dist[arc.target] = dExplo
                        parentArc[arc.target] = arc
        return t
    
    
    def Digraph(self, lspDisplay = False, name=None):
        """graph.Digraph()
        Construit la représentation svg de graph."""
        
        if name == None:
            name = './figures/'+self.info["name"]
            
        diag = gv.Digraph(f'{self.info["name"]}', filename=f'{name}', format='svg')
        
        lsp = Path(set(), Node(), set())
        if lspDisplay:
            lsp = self.LongestShortestPath()
        
        for arc in self.arcs-lsp.arcs:
            diag.edge(arc.source.name, arc.target.name, label=str(arc.weight))
        for node in self.nodes-lsp.nodes:
            diag.node(node.name, shape = 'circle')
            
        for arc in lsp.arcs:
            diag.edge(arc.source.name, arc.target.name, label=str(arc.weight), color='red')
        for node in lsp.nodes:
            diag.node(node.name, shape = 'circle', color='red')
        
        diag.attr(ratio='0.6')
        diag.attr(rankdir='LR')
        return diag
    
    def Display(self, lspDisplay = False, name=None):
        """graph.Display()
        Affiche une représentation en svg de graph."""
        if name == None:
            name = './figures/'+self.info['name']
        diag = self.Digraph(lspDisplay=lspDisplay, name=name)
        diag.view()

    def Render(self, lspDisplay=False, name=None):
        """graph.Render()
        Sauvegarde une représentation en png de graph."""
        if name == None:
            name = './figures/'+self.info["name"]
        diag = self.Digraph(lspDisplay=lspDisplay, name=name)
        diag.format = 'png'
        diag.render()
        
    def LongestShortestPath(self):
        """graph.LongestShortestPath()
        Renvoie le plus long plus court chemin entre deux sommets d'une même partie connexe du graphe orienté."""
        trees = {node0:g.Dijkstra(node0) for node0 in g.nodes}
        longestShortestPath = max([max([trees[node0].PathTowards(node1) for node1 in trees[node0].nodes]) for node0 in g.nodes])
        return longestShortestPath
    
    def AdjacencyMatrix(self, BASE):
        """graph.AdjacencyMatrix(self)
        Renvoie la matrice des plus courts chemins entre deux sommets."""
        trees = {node0:g.Dijkstra(node0) for node0 in g.nodes}
        mat = [['']+[node1.name for node1 in BASE]]
        for node0 in BASE:
            row = [node0.name]
            for node1 in BASE:
                path = trees[node0].PathTowards(node1)
                if path.IsTrivial() or path.IsNot():
                    row.append('-')
                else:
                    row.append(path.Length())
            mat.append(row)
        return mat
    
class Tree(Graph):
#---Dunder methods
    def __init__(self, nodes, arcs, name='Arbre'):
        super().__init__(nodes=nodes, arcs=arcs, name=name)
        
#---Custom methods
    def Depth(self, node):
        """tree.Depth(node)
        Renvoie la profondeur de node dans tree."""
        if not node in self.nodes:
            #Par convention, la longueur d'un chemin inexistant est l'infini
            depth = float('inf')
        else:
            depth = 0
            while len(node.ArcsTowards(self)) == 1:
                arc = list(node.ArcsTowards(self))[0]
                node = arc.source   
                depth += arc.weight
        return depth
        
    def Root(self):
        """tree.Root()
        Renvoie le sommet racine de tree."""   
        #La formule est définie car l'arbre est connexe sans cycle
        return [node for node in self.nodes if len(node.ArcsTowards(self)) == 0][0]
    
    def Digraph(self, lspDisplay = False, name=None):
        """tree.Digraph()
        Construit la représentation svg de tree."""        
       
        if name == None:
            name = self.info['name']
        diag = gv.Digraph(f'{name}', filename=f'figures/{name}', format='svg')

        for arc in self.arcs:
            diag.edge(arc.source.name, arc.targetS, label=str(arc.weight))
        for node in self.nodes:
            diag.node(node.name, shape = 'circle')
        diag.node(self.Root().name, shape='doublecircle')
        
        diag.attr(ratio='0.8')
        return diag
    
    def PathTowards(self, node):
        """tree.PathTowards(node)
        Renvoie le chemin connectant la racine de tree à node."""
        path = Path(nodes = set(), start = node, arcs = set(), name=f'Chemin_extrait_de_{self.info["name"]}_vers_{node.name}')
        #Construit le chemin s'il existe
        if node in self.nodes:
            path.AddNode(node)
            path.start = self.Root()
            while len(node.ArcsTowards(self)) == 1:
                arc = list(node.ArcsTowards(self))[0]
                node = arc.source
                path.AddNode(node)
                path.AddArc(arc)
        #Crée un chemin inexistant sinon
        else:
            path.start = self.Root()
            path.nodes = {node, self.Root()}
            path.arcs = set()
        return path

class Path(Graph):
#---Dunder methods
    def __init__(self, nodes, start, arcs, name='Chemin'):
        super().__init__(nodes=nodes, arcs=arcs, name=name)
        self.start = start
        
    def __lt__(self, other):
        if self.IsTrivial():
            return True
        elif other.IsTrivial():
            return False
        elif self.IsNot():
            return False
        elif other.IsNot():
            return True
        else:
            return self.Length() < other.Length()
    
    def __add__(self, other):
        """Somme de deux graphes comme la réunion
        des sommets et la différence des arêtes."""
        #/!\ Il faut que le sommet à l'extrémité de self soit le sommet de départ de other
        p = Path(set, Node(), set())
        p.info['name'] = self.info['name'] + '-' + other.info['name']
        p.nodes = self.nodes | other.nodes
        p.arcs = self.arcs | other.arcs
        p.start = self.start
        return p
        
        
#---Custom methods
    def Length(self):
        """path.Length()
        Renvoie la longueur de path."""
        if self.IsNot():
            return float('inf')
        else:
            return sum([arc.weight for arc in self.arcs])        
    
    def Digraph(self, lspDisplay = False, name=None):
        """path.Digraph()
        Construit la représentation svg de path."""
        
        if name == None:
            name = self.info['name']   
        diag = gv.Digraph(f'{name}', filename=f'figures/{name}', format='svg')

        
        for arc in self.arcs:
            diag.edge(arc.source.name, arc.target.name, label=str(arc.weight))
        for node in self.nodes:
            diag.node(node.name, shape = 'circle')
        diag.node(self.start.name, shape='doublecircle')
        
        diag.attr(ratio='0.8')
        return diag
    
    def EndingNode(self):
        """path.EndingNode()
        Renvoie le sommet à l'extrémité de path."""
        if self.IsNot():
            return list(self.nodes-{self.start})[0]
        else:
            return [node for node in self.nodes if len(node.ArcsFrom(self)) == 0][0]
    
    def IsTrivial(self):
        """path.IsTrivial()
        Renvoie True si path connecte un sommet à lui même
        sans l'intermédiaire d'arêtes."""
        return len(self.nodes)==1 and len(self.arcs) == 0
    
    def IsNot(self):
        """path.IsNot()
        Renvoie True si path n'est pas connexe (par convention s'il n'existe pas)."""
        return len(self.nodes)==2 and len(self.arcs) == 0
        
class Parser:
#---Dunder methods    
    def __init__(self):
        "Initialise le pattern du parser"
        
        #Généralise la notion de paramètre
        parametrePattern = pp.Group(pp.Word(pp.alphanums + '_') + pp.Suppress('="') + pp.Word(pp.alphanums + '_' + '.' + ':' + '-' + ' ') + pp.Suppress('"'))
    
        #Parse une structure de type <SOMMETS> </SOMMETS>
        sommetPattern = pp.Word(pp.alphas)
        sommetsPattern = pp.Group(pp.Suppress("<SOMMETS>") + pp.ZeroOrMore(sommetPattern + pp.Suppress(';')) + pp.Suppress("</SOMMETS>"))
        
        #Parse une structure de type <ARCS> </ARCS>
        arcPattern = pp.Group(sommetPattern + pp.Suppress(':') + sommetPattern + pp.Suppress(':') + pp.Word(pp.nums))
        arcsPattern = pp.Group(pp.Suppress("<ARCS>") + pp.ZeroOrMore(arcPattern + pp.Suppress(';')) + pp.Suppress("</ARCS>"))
        
        #Parse les parametres d'une structure de type <GRAPHE> </GRAPHE>
        headingPattern = pp.Suppress('<GRAPHE') + pp.Group(pp.ZeroOrMore(parametrePattern + (pp.Suppress(',') | pp.Suppress(pp.Empty())))) + pp.Suppress('>')
        #Détecte la fin de la structure <GRAPHE> </GRAPHE>
        tailPattern = pp.Suppress("</GRAPHE>")
        
        #Parse une structure de type <GRAPHE> </GRAPHE>
        self.graphePattern = headingPattern + sommetsPattern + arcsPattern + tailPattern

 #---Custom methods       
    def Parse(self, fileName: str):
        "Parse a <GRAPHE> </GRAPHE> structure encoded in the '../fileName' file"
        #ATTENTION: On suppose que le fichier en question est bien encodé et ne contient qu'un graphe
        retourParser = None
        try:
            retourParser = self.graphePattern.parseFile(fileName, parseAll=True)
        except pp.ParseException as err:
            print(err.line)
            print(" "*(err.column-1) + "^")
            print(err)
        return retourParser

def MatrixToStr(mat):
    """MatrixToStr(mat)
    Renvoie une str représentant mat."""
    txt = ''
    for row in mat:
        for elt in row:
            if type(elt)==str:
                txt += f'{elt:^5}'
            else:
                txt += f'{elt:^5.1f}'
        txt+='\n'
    return txt

#-----------------------[Code HTML]-------------------
#Un elt html sera ici représenté par une liste de lignes,
#et construit à l'aide de la fonction Build
def Build(balise, content='', params={}, standAlone=False, alinea=True):
    """Build(balise, content, params, standAlone, alinea)
    -> balise: texte de la balise
    -> content: liste de lignes, ou ligne
    -> params: dictionnaire de paramètres
    -> standAlone: booléen indiquant si balise unique
    -> alinea: booléen de mise en forme"""
    if type(content) != list:
        content = [content]
    
    if params != {}:
        paramsTxt = ' '+' '.join(['='.join(param) for param in params.items()])
    else:
        paramsTxt = ''
        
    
    if standAlone:
        return [f'<{balise}{paramsTxt}>']
    elif alinea or len(content) > 1:
        return [f'<{balise}{paramsTxt}>']+['    ' + line for line in content]+[f'</{balise}>']
    else:
        return [f'<{balise}{paramsTxt}>{content[0]}</{balise}>']

def BuildList(balise, liste, alinea=False):
    htmlList = []
    for elt in liste:
        if type(elt)==float:
            elt = f'{elt:^.1f}'
        htmlList += Build(balise, elt, alinea=alinea)
    return htmlList

def MatrixToTable(mat, title='Tableau'):
    """MatrixToTable(mat)
    Renvoie une str encodant en table HTML mat."""
    content = Build('caption', title)
    for row in mat:
        htmlRow = BuildList('td', row, alinea=False)
        content += Build('tr', htmlRow)
    html = Build('table', content)
    return html

def CreatePage(graph):
    """CreatePage(graph)
    Créée la page associée à graph."""
    BASE = [n for n in g.nodes]
    BASE.sort()
    g.Render(True)
    path = g.LongestShortestPath()
    
    
    #CONSTRUCTION DE HEAD    
    head = Build('title', graph.info['name'])
    head += Build('link', params={'rel': '"stylesheet"', 'href':'"style.css"'}, standAlone = True)
    
    #CONSTRUCTION DE BODY
    #---Partie1
    partie1 = Build('h2', r'1.\Détails:')
    mat = [[key+':', graph.info[key]] for key in graph.info]
    partie1 += MatrixToTable(mat, title=f"Détails de {graph.info['name']}")
    #---Partie2
    partie2 = Build('h2', r'2.\Analyse graphique:')
    image = Build('img', params={"src":f'"../figures/{graph.info["name"]}.png"',
                                 "alt":f'"Représentation de {graph.info["name"]}"',
                                 "title":f'"{graph.info["name"]}"',
                                 "class":'"shadowed"'}, standAlone = True)
    partie2 += image
    #---Partie3
    partie3 = Build('h2', r"3.\Matrice d'adjacence:")
    mat = g.AdjacencyMatrix(BASE)
    rows = Build("caption", f"Table d'adjacence de {graph.info['name']}")
    rows += Build('thead', BuildList('th', mat[0]))
    for i in range(1, len(mat)):
        row = []
        for j in range(len(mat[i])):
            if j == 0:
                row += Build('th', mat[i][j], alinea=False)
            elif BASE[i-1]==path.start and BASE[j-1]==path.EndingNode():
                row += Build('td', mat[i][j], params={"class":'"highlight"'}, alinea=False)
            else:
                row += Build('td', mat[i][j], alinea=False)
        rows += Build('tr', row)
    table = Build('table', rows, params={"class":'"shadowed"'})
    partie3 += table
    
    body = Build('h1', f'Etude de {graph.info["name"]}')
    body += Build('div', partie1, {'class': '"partie"'})
    body += Build('div', partie2, {'class': '"partie"'})
    body += Build('div', partie3, {'class': '"partie"'})
    
    #CONSTRUCTION DE LA PAGE
    page = Build('head', head) + Build('body', body)
    HTML = Build('DOCTYPE', 'html', standAlone = True) + Build('html', page)
    return HTML
    

if __name__ == "__main__":
    p = Parser()
    g = Graph(set(), set())
    
    g.FromFile(p, './ressources/graphe.txt')
    BASE = [n for n in g.nodes]
    BASE.sort()
    print(f"----------[Matrice d'adjacence de {g.info['name']}]----------")
    g.Render()
    print(MatrixToStr(g.AdjacencyMatrix(BASE)))
    
    HTML = CreatePage(g)
    with open(f"./website/{g.info['name']}.html", 'w') as file:
        for line in HTML:
            file.write(line+'\n')

    
    g.FromFile(p, './ressources/cycle.txt')
    BASE = [n for n in g.nodes]
    BASE.sort()
    print(f"----------[Matrice d'adjacence de {g.info['name']}]----------")
    print(MatrixToStr(g.AdjacencyMatrix(BASE)))
    g.Render()
    HTML = CreatePage(g)
    with open(f"./website/{g.info['name']}.html", 'w') as file:
        for line in HTML:
            file.write(line+'\n')
    
    
    
    
    
    
    
    
    
    
    
    
    #POUR D'AUTRES PARTIES (NOTAMMENT NOEUDS INTERESSANTS)
    
    # g.Render(lspDisplay=True)
    
    # #CLOSEST NODE TO EVERY POINT (totDist min)
    # trees = {node:g.Dijkstra(node) for node in BASE}
    # totDists = {node0: sum([trees[node0].Depth(node1) for node1 in BASE]) for node0 in BASE}
    
    # mini = (BASE[0], totDists[BASE[0]])
    # for node in BASE[1:]:
    #     if mini[1] > totDists[node]:
    #         mini = (node, totDists[node])
    # trees[mini[0]].Display()
    
    # #CLOSEST NODE TO EVERY POINT (maxDist min)
    # trees = {node:g.Dijkstra(node) for node in BASE}
    # maxDists = {node0: max([trees[node0].Depth(node1) for node1 in BASE]) for node0 in BASE}
  
    # mini = (BASE[0], maxDists[BASE[0]])
    # for node in BASE[1:]:
    #     if mini[1] > maxDists[node]:
    #         mini = (node, maxDists[node])
    # trees[mini[0]].Display()
    

