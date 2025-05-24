import pygame as py
import os, time


class Node():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.focused = False
        self.explored = False
        self.explorable = True
        self.neighbours = []
        self.color = None
        self.g = 0
        self.h = 0

    def __str__(self):
        return f'{self.x,self.y}'


    def add_neighbour(self, node):
        self.neighbours.append(node)
   
    def suppr(self):
        for neighbour in self.neighbours:
            neighbour.neighbours.remove(self)


class Screen():
    def __init__(self):
        self.grid_dimensions = (10,10)
        self.screen_dimensions = (600, 600)
        self.window = py.display.set_mode(self.screen_dimensions)
        self.black = (59, 54, 67)
        self.border_width = 1


        self.init()


    def init(self):
        self.window.fill(self.black)
        self.results = []
        self.sleep = 10e-2
        self.build()
        self.draw()
   
    def reset(self):
        self.window.fill(self.black)
        self.results = []
        for node in self.nodes:
            node.g = 0
            node.h = 0
            node.explored = False
            node.focused = False
        self.draw()


   
    def sleep_speed(self,e):
        self.sleep *= 10 ** (-e)
        print(f'Speed : {self.sleep}')


    def build(self):
        self.grid = []
        self.nodes = []
        for i in range(self.grid_dimensions[0]):
            row = []
            for j in range(self.grid_dimensions[1]):
                node = Node(i,j)
                self.nodes.append(node)
                row.append(node)
            self.grid.append(row)


        for x in range(self.grid_dimensions[0]):
            for y in range(self.grid_dimensions[1]):
                node = self.grid[x][y]
                for i in range(x-1,x+2):
                    for j in range(y-1, y+2):
                        if 0 <= i < self.grid_dimensions[0] and 0 <= j < self.grid_dimensions[1] and (x,y) != (i,j):
                            node.add_neighbour(self.grid[i][j])
       
        self.start = self.grid[2][2]
        self.end = self.grid[-2][-2]
       
    def click(self):
        x,y = py.mouse.get_pos()
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        i,j = x // width, y // height
        node = self.grid[i][j]
        node.explorable = not node.explorable
        self.draw_rect(node)


    def draw_rect(self, node):
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        x,y = node.x * width, node.y * height
       
        if node == self.start:
            color = (255,159,28)
        elif node == self.end:
            color = (34,81,255)
        elif node.focused:
            color = (133,159,255)
        elif node.explored:
            color = (178,196,255)
        elif node.explorable:
            color = (255,255,255)
        else:
            color = (59, 54, 67)
        py.draw.rect(self.window, color, (x+self.border_width, y+self.border_width, width - 2*self.border_width, height -2*self.border_width))
       
        py.font.init()
        font = py.font.SysFont('démo/Product-Sans-Regular.ttf', 30)
        if node.g > 0:
            text_surface = font.render(str(round(node.g,1)), False, self.black)
            self.window.blit(text_surface, (x + 2*self.border_width, y + 4* self.border_width))
        if node.h > 0:
            text_surface = font.render(str(round(node.h,1)), False, self.black)
            self.window.blit(text_surface, (x + 2*self.border_width, y + height - 25))
        py.display.flip()




    def draw(self):
        width = int(self.screen_dimensions[0] / self.grid_dimensions[0])
        height = int(self.screen_dimensions[1] / self.grid_dimensions[1])
        for i in range(self.grid_dimensions[0]):
            for j in range(self.grid_dimensions[1]):
                x, y = i * width, j * height
                node = self.grid[i][j]
                node.box = [x,y,width,height]
                self.draw_rect(node)
        py.display.flip()


    def draw_path(self):
        for result in self.results:
            for k in range(len(result['path'])-1):
                pointA = result['path'][k]
                pointB = result['path'][k+1]
                xA,yA = pointA.box[0] + pointA.box[2]/ 2 , pointA.box[1] + pointA.box[3]/ 2
                xB,yB = pointB.box[0] + pointB.box[2]/ 2 , pointB.box[1] + pointB.box[3]/ 2
                py.draw.line(self.window,result['color'],(xA,yA),(xB,yB), width=10)
        py.display.flip()


    def distance(self, nodeA, nodeB):
        return ((nodeA.x - nodeB.x)**2 + (nodeA.y - nodeB.y)**2)** 0.5


    def dijkstra(self):
        self.screenshot()
        i = self.dijkstra_algo_iterations(self.start, self.end)
        self.reset()
        n = 3
        iterations = [i * k // n for k in range(n)]
        distance = self.dijkstra_algo(self.start, self.end, iterations)
        total = distance[self.end]['distance']
        node = self.end
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
        self.results.append({'algo':'Dijkstra','distance':int(total),'color':(255,159,28),'path':path})
        self.draw_path()
        self.screenshot()


    def dijkstra_algo_iterations(self, start, end):
        queue = [start]
        distance = {start:{'distance':0,'source':None}}
        t = time.time()
        i = 0
        while queue:
            i += 1
            current = queue.pop(0)
            current.focused = True
            if current == end:
                print(f'Dijkstra time : {time.time() - t}s')
                return i
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        queue.append(neighbour)
                        neighbour.explored = True
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance':new_dist,'source':current}
                        neighbour.g = new_dist


    def dijkstra_algo(self, start, end, iterations):
        queue = [start]
        distance = {start:{'distance':0,'source':None}}
        i = 0
        while queue:
            i += 1
            current = queue.pop(0)
            current.focused = True
            self.draw_rect(current)
            if i in iterations:
                self.screenshot()
            if current == end:
                return distance
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        queue.append(neighbour)
                        neighbour.explored = True
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if (neighbour not in distance or new_dist < distance[neighbour]['distance']):
                        distance[neighbour] = {'distance':new_dist,'source':current}
                        neighbour.g = new_dist
                        self.draw_rect(neighbour)
            time.sleep(self.sleep)
        print(i)
        return None


    def a_star(self):
        self.screenshot()
        i = self.a_star_algo_iterations(self.start, self.end)
        print(i)
        self.reset()
        n = 3
        iterations = [i * k // n for k in range(n)]
        print(iterations)
        distance = self.a_star_algo(self.start, self.end, iterations)
        path = []
        node = self.end
        total = distance[self.end]['distance']
        path = [node]
        while node != self.start:
            node = distance[node]['source']
            path.append(node)
        self.results.append({'algo':'A*','distance':int(total),'color':(255,159,28),'path':path})
        self.draw_path()
        self.screenshot()


    def a_star_algo_iterations(self, start, end):
        queue = [{'node':start,'priority':0}]
        distance = {start:{'distance':0,'source':None}}
        i = 0
        t = time.time()
        while queue:
            i += 1
            current = queue.pop(0)['node']
            if current == end:
                print(t, time.time())
                print(f'A* time : {time.time() - t}s')
                return i
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        neighbour.explored = True
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance':new_dist,'source':current}
                        neighbour.g = new_dist
                        h = self.distance(neighbour, self.end)
                        neighbour.h = h
                        priority = new_dist + h
                        queue.append({'node':neighbour,'priority':priority})
                    queue.sort(key=lambda x:x['priority'])


    def a_star_algo(self, start, end, iterations):
        queue = [{'node':start,'priority':0}]
        distance = {start:{'distance':0,'source':None}}
        i = 0
        while queue:
            i += 1
            current = queue.pop(0)['node']
            current.focused = True
            self.draw_rect(current)
            if i in iterations:
                self.screenshot()
            if current == end:
                return distance
            for neighbour in current.neighbours:
                if neighbour.explorable:
                    if not neighbour.explored:
                        neighbour.explored = True
                    new_dist = distance[current]['distance'] + self.distance(current, neighbour)
                    if neighbour not in distance or new_dist < distance[neighbour]['distance']:
                        distance[neighbour] = {'distance':new_dist,'source':current}
                        neighbour.g = new_dist
                        h = self.distance(neighbour, self.end)
                        neighbour.h = h
                        priority = new_dist + h
                        self.draw_rect(neighbour)
                        queue.append({'node':neighbour,'priority':priority})
                    queue.sort(key=lambda x:x['priority'])
            time.sleep(self.sleep)
        print(i)
        return None


    def screenshot(self):
        path = 'démo/screenshots/'
        py.image.save(self.window, f'{path}/image_{len(os.listdir(path))}.png')


screen = Screen()
running = True
while running:
    for event in py.event.get():
        if event.type == py.QUIT:
            running = False
        if event.type == py.MOUSEBUTTONDOWN:
            mouse_presses = py.mouse.get_pressed()
            if mouse_presses[0]:
                screen.click()
        if event.type == py.KEYDOWN:
            if event.key == py.K_r:
                screen.reset()
            if event.key == py.K_d:
                screen.dijkstra()
            if event.key == py.K_a:
                screen.a_star()
            if event.key == py.K_UP:
                screen.sleep_speed(1)
            if event.key == py.K_DOWN:
                screen.sleep_speed(-1)