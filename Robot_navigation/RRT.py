import math, sys, pygame, random
from math import *
from pygame import *
import time
class Node(object):
    def __init__(self,point,Parent_node):
        super(Node, self).__init__()
        self.parent_pointer = Parent_node
        self.point = point


def reset():
    global node
    screen.fill(white)
    node = []
    pygame.draw.polygon(screen, maroon, PointList, 0)
    text = font2.render("Rapidly-exploring", 1, (10, 10, 10))
    screen.blit(text, (BORDER,10))
    text = font2.render("Random Tree", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 40))
    text = font2.render("> Click to Insert the", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 70))
    text = font2.render("Start and End Node", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 100))
    text = font2.render("> Press Left Mouse Button", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 130))
    text = font2.render("For Inserting Vertices ", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 160))
    text = font2.render("of Polygon", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 190))
    text = font2.render("> Press Right Mouse Button", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 220))
    text = font2.render("to complete the Polygon", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 250))
    text = font2.render("> Press Middle Mouse ", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 280))
    text = font2.render("Button or Any Key to", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 310))
    text = font2.render("Start RRT", 1, (10, 10, 10))
    screen.blit(text, (BORDER, 340))


def generate_rrt():
    global node
    while True:
        x_rand = Random_config()
        x_near = nearest_neighbour(x_rand)
        u = select_state(x_rand,x_near)
        if check_collision_polygon(u.point) is None:
            pygame.draw.line(screen, lineColor, x_near.point, u.point)
            pygame.draw.polygon(screen, maroon, PointList, 0)
            text = font2.render('No. of Nodes:', 1, (10, 10, 10))
            screen.blit(text, (BORDER+5, 0.15*YDIM))
            text = font.render(str(len(node)), 1, (10, 10, 10))
            screen.blit(text, (BORDER+150, 0.15 * YDIM))
            text = font2.render('Time:', 1, (10, 10, 10))
            screen.blit(text, (BORDER+5, 0.4 * YDIM))
            text = font.render(str(((float("{0:.4f}".format(time.time() - start_time))))), 1, (10, 10, 10))
            screen.blit(text, (BORDER+70, 0.4*YDIM))
            text = font.render('s', 1, (10, 10, 10))
            screen.blit(text, (BORDER + 170, 0.4 * YDIM))
            return u


def check_collision_polygon(point):
    for i in range(0,len(polygon_list)):
        vertex = polygon_list[i]
        if ray_casting(point,vertex)==1:
            return 1


def ray_casting(point,vertex):
    count = 0
    for i in range(0,len(vertex)):
        if i != (len(vertex)-1):
            if vertex[i][1] < vertex[i+1][1]:
                count += ray_intersect(point,vertex[i],vertex[i+1])
            else:
                count += ray_intersect(point, vertex[i+1], vertex[i])
        else:
            if vertex[i][1] < vertex[0][1]:
                count += ray_intersect(point,vertex[i],vertex[0])
            else:
                count += ray_intersect(point, vertex[0], vertex[i])
    if count % 2 == 0:
        return 0
    else:
        return 1


def ray_intersect(P,A,B):
    if P[1] == A[1] or P[1] == B[1]:
        P[1] += 10
    if P[1] < A[1] or P[1] > B[1]:
        return 0
    elif P[0] > max(A[0],B[0]):
        return 0
    else:
        if P[0] < min(A[0],B[0]):
            return 1
        else:
            if A[0] != B[0]:
                segment_slope = (B[1]-A[1])/(B[0]-A[0])
            else:
                segment_slope = inf
            if A[0]!=P[0]:
                ray_slope = (P[1] - A[1])/(P[0] - A[0])
            else:
                ray_slope = inf
            if segment_slope < ray_slope:
                return 1
            else:
                return 0


def draw_path(u):
    while u.parent_pointer!=None:
        pygame.draw.line(screen, GoalPathColor,u.point,u.parent_pointer.point)
        u = u.parent_pointer


def check_goal(u):
    if dist(u.point,goalNode.point) <= RADIUS:
        return True
    else:
        return False


def Check_start_state_collide(u):
    if dist(initialNode.point,u) <= RADIUS:
        return True
    else:
        return False


def Random_config():
    while True:
        p = random.random()*BORDER, random.random()*YDIM
        if Check_start_state_collide(p)!=True:
            break
    return p


def select_state(x_rand,near_node):
    if dist(x_rand,near_node.point) <= delta:
        newNode = Node(x_rand,near_node)
        return newNode
    theta = atan2((x_rand[1]-near_node.point[1]),(x_rand[0] - near_node.point[0]) )
    point = cos(theta)*delta + near_node.point[0], sin(theta)*delta + near_node.point[1]
    newNode = Node(point,near_node)
    return newNode


def nearest_neighbour(x_rand):
    min_dist = dist(node[0].point,x_rand)
    near_node = node[0]
    for p in node:
        if dist(p.point,x_rand)<=min_dist:
            min_dist = dist(p.point,x_rand)
            near_node = p
    return near_node


def dist(p1,p2):
    return sqrt((p1[1] - p2[1])**2 + (p1[0] - p2[0])**2)


def border_check(p):
    if p[0] < BORDER:
        return True
    else:
        return False


XDIM = 1030
YDIM = 500
BORDER = 720
windowsize = [XDIM,YDIM]
delta = 10.0
RADIUS = 10
SMALL_RADIUS = 2
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowsize)
white = 255, 255, 255
black = 0, 0, 0
StartNodeColor = 255, 0, 0
blue = 0, 0, 255
GoalNodeColor = 0, 255, 0
lineColor = 0, 180, 105
gray = 128, 128, 128
maroon = 192, 192, 192
GoalPathColor = 0, 0, 0
node = []
goalNode = Node(None,None)
initialNode = Node(None,None)
polygon_list = []
font = pygame.font.Font('C:\Windows\Fonts\Calibri.ttf', 30)
font2 = pygame.font.Font('C:\Windows\Fonts\Calibri.ttf', 25)

PointList = []
PointList.append((BORDER, 0))
PointList.append((XDIM, 0))
PointList.append((XDIM, YDIM))
PointList.append((BORDER, YDIM))
start_time = 0

def main():
    reset()
    K = 10000
    done_flag=0
    global goalNode
    global initialNode
    global start_time
    currentstate = 'init'
    initPose = False
    finalPose = False
    vertices = []
    global polygon_list
    while True:
        if currentstate == 'StartRRT':
            if K > 0:
                u = generate_rrt()
                K-=1
                node.append(u)
                if check_goal(u):
                    draw_path(u)
                    currentstate = "Found RRT"
                    print("Found")
            else:
                if done_flag==0:
                    text = font.render('Uggh ran out of nodes!!', 1, (10, 10, 10))
                    screen.blit(text, (BORDER + 10 , 0.6 * YDIM))
                    done_flag = 1
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            elif e.type == KEYUP and (currentstate == 'SetPolygon'):
                start_time = time.time()
                if not vertices:
                    currentstate = 'StartRRT'
                else:
                    polygon_list.append(vertices)
                    pygame.draw.polygon(screen, gray, vertices, 0)
                    pygame.display.update()
                    currentstate = 'StartRRT'
            if e.type == MOUSEBUTTONDOWN:
                if currentstate == 'init':
                    if initPose == False:
                        if border_check(e.pos)==True:
                            initialNode = Node(e.pos,None)
                            pygame.draw.circle(screen,StartNodeColor,initialNode.point,RADIUS)
                            initPose = True
                            node.append(initialNode)
                    elif finalPose == False:
                            if border_check(e.pos)==True:
                                goalNode = Node(e.pos,None)
                                pygame.draw.circle(screen,GoalNodeColor,goalNode.point,RADIUS)
                                currentstate = 'SetPolygon'
                                finalPose = True
                elif currentstate == 'SetPolygon' or currentstate=='check':
                    button = pygame.mouse.get_pressed()
                    if button[0] == 1:
                        if border_check(e.pos)==True:
                            vertices.append(e.pos)
                            pygame.draw.circle(screen, gray, e.pos, SMALL_RADIUS)
                    elif button[2] == 1 :
                        if vertices:
                            if len(vertices)>2:
                                polygon_list.append(vertices)
                                pygame.draw.polygon(screen,gray,vertices,0)
                                vertices = []
                            else:
                                vertices = []
                    elif button[1] == 1:
                        start_time = time.time()
                        if not vertices:
                            currentstate = 'StartRRT'
                        else:
                            if len(vertices)>2:
                                polygon_list.append(vertices)
                                pygame.draw.polygon(screen,gray,vertices,0)
                                vertices = []
                            else:
                                vertices = []
                            currentstate = 'StartRRT'
                else:
                    currentstate = 'init'
                    initPose = False
                    finalPose = False
                    vertices = []
                    K = 10000
                    done_flag = 0
                    polygon_list = []
                    reset()
        pygame.display.update()
        fpsClock.tick(1000)
            
  
if __name__ == '__main__':
   main()
    
