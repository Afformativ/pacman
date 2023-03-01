from abc import ABC, abstractmethod

def get_neighbors(X):
    return [(0, 0), (1, 1)]

class PathFinder:
    @abstractmethod
    def find_path(self, start, goal):
        pass

    def eval(self, path):
        return len(path)

class DFS(PathFinder):
    def find_path(self, start, goal):
        stack = [start]
        curr=start
        parents={}
        visited = set()
        while curr != goal and stack:
            curr = stack.pop()
            visited.add(curr)
            derivatives = get_neighbors(curr)
            for derivative in derivatives:
                if not derivative in visited:
                    parents[derivative]=curr
                    stack.append(derivative)
        path=[]
        curr=goal
        while parents[curr]!=start:
            path.append(curr)
            curr=parents[curr]
        return path.reverse()
            
        
class BFS(PathFinder):
    def find_path(self, start, goal):
        queue = [start]
        curr=start
        parents={}
        visited = set()
        while curr != goal and queue:
            curr = queue.pop()
            visited.add(curr)
            derivatives = get_neighbors(curr)
            for derivative in derivatives:
                if not derivative in visited:
                    parents[derivative]=curr
                    queue.insert(0, derivative)
        path=[]
        curr=goal
        while parents[curr]!=start:
            path.append(curr)
            curr=parents[curr]
        return path.reverse()
        
# uniform cost search, same as BFS
class UCS(BFS):
    pass

# Iterative deepening depth-first search
class IDFS(PathFinder):
    def find_path(self, start, goal):
        raise NotImplemented()

# bidirectional search
class BDS(PathFinder):
    def find_path(self, start, goal):
        raise NotImplemented()
