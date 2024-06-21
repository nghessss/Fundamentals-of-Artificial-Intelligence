import numpy as np
from queue import PriorityQueue

def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
    
    path=[]
    visited={}
    queue = [(start,None)]
    while queue:
        last = queue.pop(0)
        
        node = last[0]
        if node in visited:
            continue
        visited[node] = last[1]
        
        for i in range(len(matrix[node])):
            if matrix[node][i] != 0 and i not in visited :
                queue.append((i,node))
                
                if i == end:
                    path.append(i)
                    path.append(node)
                    while visited[node] != None:
                        path.append(visited[node])
                        node = visited[node]
                    path.reverse()
                    print(visited)
                    print(path)
                    return visited, path
                
    return visited, path

def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    
    path=[]
    visited={}
    visited[start] = None
    path.append(start)
    DFS_recursive(matrix, start, end, visited, path)   
    visited.pop(end)
    print('visited',visited)
    print('path',path)
    return visited, path

def DFS_recursive(matrix, start, end, visited, path):
    """
    DFS algorithm
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    visited: dictionary
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    """
    n = len(matrix)
    for i in range(n):
        if matrix[start][i] != 0 and i not in visited:
            visited[i] = start
            path.append(i)
            if i == end:
                return
            DFS_recursive(matrix, i, end, visited, path)
            if end in visited:
                return
            path.pop()
            
    


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    # priority queue
    queue = PriorityQueue()
    queue.put((0, (start, None)))
    while not queue.empty():
        print('queue',queue.queue)
        print('visited',visited)
        

        cost, top = queue.get()
        print('top',top)
        node = top[0]
        parent = top[1]
        if (node in visited):
            continue
        visited[node] = parent
        if (node == end):
            path.append(node)
            while visited[node] != None:
                node = visited[node]
                path.append(node)
            path.reverse()
            print('visited',visited)
            print('path',path)
            return visited, path
        if node == end:
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] != 0 and i not in visited:
                queue.put((cost + matrix[node][i], (i, node)))
    return visited, path

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}
    # priority queue
    queue = PriorityQueue()
    queue.put((0, (start, None)))
    while not queue.empty():
        print('queue',queue.queue)
        print('visited',visited)
        

        _, top = queue.get()
        print('top',top)
        node = top[0]
        parent = top[1]
        if (node in visited):
            continue
        visited[node] = parent
        if (node == end):
            path.append(node)
            while visited[node] != None:
                node = visited[node]
                path.append(node)
            path.reverse()
            print('visited',visited)
            print('path',path)
            return visited, path
        if node == end:
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] != 0 and i not in visited:
                queue.put((matrix[end][i], (i, node)))
    return visited, path
def heuristic_Astar(pos, current, Goal):
    h = (pos[current][0] - pos[Goal][0])**2 + (pos[current][1] - pos[Goal][1])**2
    return h
    
    
def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 

    path=[]
    visited={}
    # priority queue
    queue = PriorityQueue()
    queue.put((0, (start, None, 0)))
    while not queue.empty():
        print('queue',queue.queue)
        print('visited',visited)
        

        _, top = queue.get()
        print('top',top)
        node = top[0]
        parent = top[1]
        cost = top[2]
        if (node in visited):
            continue
        visited[node] = parent
        if (node == end):
            path.append(node)
            while visited[node] != None:
                node = visited[node]
                path.append(node)
            path.reverse()
            print('visited',visited)
            print('path',path)
            while not queue.empty():
                queue.get()
            return visited, path
        if node == end:
            break
        for i in range(len(matrix[node])):
            if matrix[node][i] != 0 and i not in visited:
                queue.put((cost + matrix[node][i] + heuristic_Astar(pos,i,end), (i, node, cost + matrix[node][i])))
    return visited, path

    

