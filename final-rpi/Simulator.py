from tkinter import *
import time
import math
max_value=700
arena_size=20
pillars = [(10,10,'R'),(10,7,'R'),(10,14,'R'),(10,18,'L')]
turn90=2
turn180=3
forward=1

def next_move(initial,final):
    if initial == 'U':
        if final == 'U':
            return 'F'
        elif final == 'L':
            return 'L'
        elif final == 'R':
            return 'R'
    elif initial == 'D':
        if final == 'L':
            return 'R'
        elif final == 'R':
            return 'L'
        elif final == 'D':
            return 'F'
    elif initial == 'L':
        if final == 'U':
            return 'R'
        elif final == 'L':
            return 'F'
        elif final == 'D':
            return 'L'
    elif initial == 'R':
        if final == 'U':
            return 'L'
        elif final == 'R':
            return 'F'
        elif final == 'D':
            return 'R'

def direction_at_node(car,node):
    if car == 'U' and node == 'R':
        return 'R'
    if car == 'U' and node == 'L':
        return 'L'
    if car == 'L' and node == 'U':
        return 'R'
    if car == 'L' and node == 'D':
        return 'L'
    if car == 'D' and node == 'L':
        return 'R'
    if car == 'D' and node == 'R':
        return 'L'
    if car == 'R' and node == 'U':
        return 'L'
    if car == 'R' and node == 'D':
        return 'R'
    return False

def astar_search(start,end,avoid):
    route=[0,start[0],start[1],start[2]] # [value,xCoord,yCoord,Orientation,steps...]
    while (route[1]!=end[0] or route[2]!=end[1]) and route[0]<max_value:
        u = route.copy()
        d = route.copy()
        l = route.copy()
        r = route.copy()
        if route[2]+1<arena_size-1 and (route[1],route[2]+1) not in avoid and route[3] != "D":
            u[2]+=1
            h=math.sqrt((u[1]-end[0])**2+(u[2]-end[1])**2)
            u[0]+=h
            u.append((route[1],route[2]+1,next_move(u[3],'U')))
            if u[3]=="L" or u[3]=="R":
                u[0]+=turn90
            elif u[3]=="D":
                u[0]+=turn180
            else:
                u[0]+=forward
            u[3]="U"
        else:
            u[0] = 1000

        if route[1]+1<arena_size-1 and (route[1]+1,route[2]) not in avoid and route[3] != "L":
            r[1]+=1
            h=math.sqrt((r[1]-end[0])**2+(r[2]-end[1])**2)
            r[0]+=h
            r.append((route[1]+1,route[2],next_move(r[3],'R')))
            if r[3]=="U" or r[3]=="D":
                r[0]+=turn90
            elif r[3]=="L":
                r[0]+=turn180
            else:
                r[0]+=forward
            r[3]="R"
        else:
            r[0] = 1000

        if route[2]-1>=1 and (route[1],route[2]-1) not in avoid and route[3] != "U":
            d[2]-=1
            h=math.sqrt((d[1]-end[0])**2+(d[2]-end[1])**2)
            d[0]+=h
            d.append((route[1],route[2]-1,next_move(d[3],'D')))
            if d[3]=="L" or d[3]=="R":
                d[0]+=turn90
            elif d[3]=="U":
                d[0]+=turn180
            else:
                d[0]+=forward
            d[3]="D"
        else:
            d[0] = 1000

        if route[1]-1>=1 and (route[1]-1,route[2]) not in avoid and route[3] != "R":
            l[1]-=1
            h=math.sqrt((l[1]-end[0])**2+(l[2]-end[1])**2)
            l[0]+=h
            l.append((route[1]-1,route[2],next_move(l[3],'L')))
            if l[3]=="U" or u[3]=="D":
                l[0]+=turn90
            elif l[3]=="R":
                l[0]+=turn180
            else:
                l[0]+=forward
            l[3]="L"
        else:
            l[0] = 1000

        if min(round(u[0],2),round(d[0],2),round(l[0],2),round(r[0],2))==round(u[0],2):
            route = u
        elif min(round(u[0],2), round(d[0],2), round(l[0],2), round(r[0],2)) == round(d[0],2):
            route = d
        elif min(round(u[0],2), round(d[0],2), round(l[0],2), round(r[0],2)) == round(l[0],2):
            route = l
        else :
            route = r
    if route[3] != end[2]:
        if direction_at_node(route[3],end[2]) != False:
            route.append((route[len(route) - 1][0], route[len(route) - 1][1],direction_at_node(route[3],end[2])))
        else:
            route.append((route[len(route) - 1][0], route[len(route) - 1][1], 'L'))
            route.append((route[len(route) - 1][0], route[len(route) - 1][1], 'L'))
    elif len(route)!=3:
        route.append((route[len(route)-1][0],route[len(route)-1][1],'F'))
    return route

def routes_generator(start,pillars,pillars_identified,nodes_identified):
    nodes = [start]
    gap = 3
    for pillar in pillars:
        if pillar[0]+gap<arena_size-1 and pillar[0]+gap not in (19,0) and pillar[1] not in (19,0) and (pillar[0]+gap,pillar[1],'L') not in nodes and pillar not in pillars_identified and (pillar[0]+gap,pillar[1],'L') not in nodes_identified:
            nodes.append((pillar[0]+gap,pillar[1],'L'))
        if pillar[1]+gap<arena_size-1 and pillar[0] not in (19,0) and pillar[1]+gap not in (19,0) and (pillar[0],pillar[1]+gap,'D') not in nodes and pillar not in pillars_identified and (pillar[0],pillar[1]+gap,'D') not in nodes_identified:
            nodes.append((pillar[0],pillar[1]+gap,'D'))
        if pillar[0]-gap>=1 and pillar[0]-gap not in (19,0) and pillar[1] not in (19,0) and (pillar[0]-gap,pillar[1],'R') not in nodes and pillar not in pillars_identified and (pillar[0]-gap,pillar[1],'R') not in nodes_identified:
            nodes.append((pillar[0]-gap,pillar[1],'R'))
        if pillar[1]-gap>=1 and pillar[1]-gap not in (19,0) and pillar[0] not in (19,0) and (pillar[0],pillar[1]-gap,'U') not in nodes and pillar not in pillars_identified and (pillar[0],pillar[1]-gap,'U') not in nodes_identified:
            nodes.append((pillar[0],pillar[1]-gap,'U'))

    avoid = []
    for pillar in pillars:
        avoid.append((pillar[0],pillar[1]))
        avoid.append((pillar[0]+1,pillar[1]))
        avoid.append((pillar[0],pillar[1]+1))
        avoid.append((pillar[0]+1,pillar[1]+1))
        avoid.append((pillar[0]-1,pillar[1]))
        avoid.append((pillar[0],pillar[1]-1))
        avoid.append((pillar[0]-1,pillar[1]-1))
        avoid.append((pillar[0]+1,pillar[1]-1))
        avoid.append((pillar[0]-1,pillar[1]+1))


    for node in range(len(nodes)-1,0,-1):
        if (nodes[node][0],nodes[node][1]) in avoid:
            nodes.remove(nodes[node])

    routes_cost = []
    routes = []
    for a in range(len(nodes)):
        routes.append([])
        routes_cost.append([])
        for b in range(len(nodes)):
            routes_cost[a].append(-1)
            routes[a].append([])
    for node_start in range(len(nodes)):
        for node_end in range(len(nodes)):
            if node_start != node_end:
                route = astar_search(nodes[node_start],nodes[node_end],avoid)
                routes_cost[node_start][node_end] = route[0]
                if route[0]>=max_value:
                    routes_cost[node_start][node_end] = -1
                routes[node_start][node_end] = route[4:]
    return (nodes,routes,routes_cost)

def exhaustive_search(cost,path,toVisit,routes_cost):
        if len(toVisit) == 0:
            return cost,path
        cheapest = 10000
        finalPath = []
        for a in range(len(toVisit)):
            tempPath = path.copy()
            tempToVisit = toVisit.copy()
            tempPath.append(tempToVisit[a])
            tempToVisit.remove(tempToVisit[a])
            search = exhaustive_search(cost+routes_cost[tempPath[len(tempPath)-1]][tempPath[len(tempPath)-2]],tempPath,tempToVisit,routes_cost)
            if search[0] < cheapest:
                finalPath = search[1]
                cheapest = search[0]
        return cheapest,finalPath

def nearest_neighbours(routes_cost):
    toVisit = list(range(1,len(routes_cost)))
    visited = [0]
    while len(toVisit) != 0:
        cheapest = max_value
        node = 0
        for a in range(len(toVisit)):
            if routes_cost[visited[len(visited)-1]][toVisit[a]] != -1 and routes_cost[visited[len(visited)-1]][toVisit[a]] <= cheapest:
                node = toVisit[a]
                cheapest = routes_cost[visited[len(visited)-1]][toVisit[a]]
        if node != 0:
            visited.append(node)
            toVisit.remove(node)
    return visited

def shortest_path(routes_cost):
    shortest = nearest_neighbours(routes_cost)
    return shortest


def pillar_direction(value):
    if value == 'U':
        return 90
    if value == 'D':
        return 270
    if value == 'L':
        return 180
    if value == 'R':
        return 0

def car_direction(value):
    if value == 'F':
        return 0
    if value == 'L':
        return 90
    if value == 'R':
        return 270

def car_direction_string(value):
    if value == 0:
        return 'R'
    if value == 90:
        return 'U'
    if value == 180:
        return 'L'
    if value == 270:
        return 'D'

def simulator_path_generator(spoint,pillars,pillars_identified,nodes_identified):
    global nodes
    simulator_path = []
    ans = routes_generator(spoint,pillars,pillars_identified,nodes_identified)
    nodes = ans[0]
    path = shortest_path(ans[2])
    for r in range(len(path)):
        if r <= len(path) - 2:
            simulator_path.extend(ans[1][path[r]][path[r + 1]])
    return simulator_path

def target_nodes_generator(pillars,pillars_direction):
    target_nodes = []
    gap = 3
    for pillar in range(len(pillars)):
        if pillars_direction[pillar] == 'U':
            target_nodes.append((pillars[pillar][0],pillars[pillar][1]+gap,'D'))
        elif pillars_direction[pillar] == 'D':
            target_nodes.append((pillars[pillar][0], pillars[pillar][1] - gap, 'U'))
        elif pillars_direction[pillar] == 'L':
            target_nodes.append((pillars[pillar][0] - gap, pillars[pillar][1], 'R'))
        elif pillars_direction[pillar] == 'R':
            target_nodes.append((pillars[pillar][0] + gap, pillars[pillar][1], 'L'))
    return target_nodes

def find_pillars_identified(target_node):
    gap = 3
    if target_node[2] == 'U':
        return (target_node[0],target_node[1]+gap)
    if target_node[2] == 'D':
        return (target_node[0],target_node[1]-gap)
    if target_node[2] == 'R':
        return (target_node[0]+gap,target_node[1])
    if target_node[2] == 'L':
        return (target_node[0]-gap,target_node[1])
# pillars = [(1,18),(6,12),(10,7),(13,2),(15,16),(19,9)]
# pillars_direction = ['D','U','R','R','L','L']
# 10,6,l 10,12,r 10,9,r 10,15,r 10,18,l 10,3,r
# 6,10,d 12,10,u 9,10,d 15,10,u 18,10,d 3,10,u

# 1,18,D 6,12,U 10,7,R 13,2,R 15,16,L 19,9,L
pillars = []
pillars_identified = []
pillars_direction = []
nodes = []
target_nodes = []

start = 0
end = 0
root = Tk()
root.title("MDP Simulator")
root.geometry("1000x810")
simulator_path = []
moves = [(1,2,'F'),(1,3,'R'),(2,3,'R'),(2,4,'R'),(2,5,'R'),(2,6,'R')]
scanned = []
scanned_tuple_form = []

def car(self,b,a,cur,lim,list,orientation):
    global display_scanned
    global pillars_identified
    global end
    global display_time
    for widgets in map.winfo_children():
        widgets.destroy()
    for f in range(20):
        for s in range(20):
            if s <= 3 and f >= 16:
                frame = Frame(map, height=30, width=30, bg="#B9B9B9", relief=RAISED, borderwidth=2)
                frame.grid(row=f, column=s)
            elif (s,19-f) in pillars_identified:
                arrow = Canvas(map, width=26, height=26,bg='#36F049')
                arrow.grid(row=f, column=s)
                arrow.create_text(14, 16, text=">", angle=pillar_direction(pillars_direction[pillars.index((s,19-f))]), anchor="w", font=('Helvetica', '15', 'bold'),fill='white')
            elif (s,19-f) in pillars:
                arrow = Canvas(map, width=26, height=26,bg='black')
                arrow.grid(row=f, column=s)
                arrow.create_text(14, 16, text=">", angle=pillar_direction(pillars_direction[pillars.index((s,19-f))]), anchor="w", font=('Helvetica', '15', 'bold'),fill='white')
            else:
                frame = Frame(map, height=30, width=30, relief=RAISED, borderwidth=2)
                frame.grid(row=f, column=s)

    x=19-a
    y=b
    map.winfo_children()[20*(x+1)+y+1].destroy()
    map.winfo_children()[20 * (x + 1) + y].destroy()
    map.winfo_children()[20 * (x + 1) + y - 1].destroy()
    map.winfo_children()[20 * (x) + y + 1].destroy()
    map.winfo_children()[20 * (x) + y].destroy()
    map.winfo_children()[20 * (x) + y - 1].destroy()
    map.winfo_children()[20 * (x-1) + y + 1].destroy()
    map.winfo_children()[20 * (x-1) + y].destroy()
    map.winfo_children()[20 * (x-1) + y - 1].destroy()
    arrow = Canvas(map, width=86, height=86, bg="#EF5938", relief=RAISED, borderwidth=0)
    arrow.grid(row=x - 1, column=y - 1, rowspan=3, columnspan=3)
    arrow.create_text(45, 45, text=">", angle=(orientation+car_direction(list[cur][2]))%360, anchor="w", font=('Helvetica', '50', 'bold'))


    if (b,a,car_direction_string((orientation+car_direction(list[cur][2]))%360)) in nodes:
        if len(scanned) == 0 or scanned[len(scanned)-1] != '('+str(b)+','+str(a)+','+car_direction_string((orientation+car_direction(list[cur][2]))%360)+')':
            scanned.append('('+str(b)+','+str(a)+','+car_direction_string((orientation+car_direction(list[cur][2]))%360)+')')
            scanned_tuple_form.append((b,a,car_direction_string((orientation+car_direction(list[cur][2]))%360)))
        display_scanned.destroy()
        display_scanned = Canvas(root, width=600, height=70, relief=RAISED, borderwidth=0)
        display_scanned.grid(row=21, column=0)
        display_scanned.create_text(10, 30, text='Scan:'+' '.join(scanned), anchor="w", font=('Helvetica', '10', 'bold'),width=600)

    if (b, a, car_direction_string((orientation + car_direction(list[cur][2])) % 360)) in target_nodes:
        pillars_identified.append(find_pillars_identified((b, a, car_direction_string((orientation + car_direction(list[cur][2])) % 360))))
        new_simulator_path = simulator_path_generator((b, a, car_direction_string((orientation + car_direction(list[cur][2])) % 360)), pillars,pillars_identified,scanned_tuple_form)
        if len(pillars) != len(pillars_identified):
            self.after(360, lambda: car(self,new_simulator_path[0][0],new_simulator_path[0][1],0,len(new_simulator_path),new_simulator_path,orientation+car_direction(list[cur][2])))
        else:
            # refresh()
            end = time.time()
            display_time.destroy()
            display_time = Canvas(root, width=150, height=70, relief=RAISED, borderwidth=0)
            display_time.grid(row=11, column=3)
            display_time.create_text(10, 30, text='Time:' + str(round(end - start,2))+'s', anchor="w",font=('Helvetica', '10', 'bold'))

    elif cur<lim-1:
        self.after(360, lambda: car(self,list[cur+1][0], list[cur+1][1],cur+1,lim,list,orientation+car_direction(list[cur][2])))

def run(self):
    global simulator_path
    global pillars
    global pillars_direction
    global target_nodes
    global scanned
    global start
    start = time.time()
    scanned = []
    target_nodes = target_nodes_generator(pillars,pillars_direction)
    simulator_path = simulator_path_generator((1,1,'U'),pillars,[],[])
    car(self,simulator_path[0][0],simulator_path[0][1],0,len(simulator_path),simulator_path,90)

def edit_pillars(choice):
    global entry
    global display_pillars
    global pillars
    global scanned
    scanned = []
    if choice == 'add':
        l = entry.get()
        pillars_input = l.split(' ')
        for pillar in pillars_input:
            pillar_info = pillar.split(',')
            pillars.append((int(pillar_info[0]),int(pillar_info[1])))
            pillars_direction.append(pillar_info[2].upper())
        entry.delete(0,END)
    elif choice == 'remove':
        if len(pillars) !=0:
            pillars.pop()
            pillars_direction.pop()
    refresh()



def refresh():
    global map

    display_pillars_text = ''
    for x in range(len(pillars)):
        display_pillars_text += ('('+str(pillars[x][0])+','+str(pillars[x][1])+','+pillars_direction[x]+')'+'  ')

    display_pillars = Canvas(root, width=600, height=50, relief=RAISED, borderwidth=0)
    display_pillars.grid(row=0, column=0)
    display_pillars.create_text(10, 30, text='Pillars:' + display_pillars_text, anchor="w",font=('Helvetica', '10', 'bold'))

    for widgets in map.winfo_children():
        widgets.destroy()
    for a in range(20):
        for b in range(20):
            if b <= 3 and a >= 16:
                frame = Frame(map, height=30, width=30, bg="#B9B9B9", relief=RAISED, borderwidth=2)
                frame.grid(row=a, column=b)
            elif (b,19-a) in pillars_identified:
                arrow = Canvas(map, width=26, height=26,bg='#36F049')
                arrow.grid(row=a, column=b)
                arrow.create_text(14, 16, text=">", angle=pillar_direction(pillars_direction[pillars.index((b,19-a))]), anchor="w", font=('Helvetica', '15', 'bold'),fill='white')
            elif (b, 19 - a) in pillars:
                arrow = Canvas(map, width=26, height=26, bg='black')
                arrow.grid(row=a, column=b)
                arrow.create_text(14, 16, text=">",
                                  angle=pillar_direction(pillars_direction[pillars.index((b, 19 - a))]), anchor="w",
                                  font=('Helvetica', '15', 'bold'), fill='white')
            else:
                frame = Frame(map, height=30, width=30, relief=RAISED, borderwidth=2)
                frame.grid(row=a, column=b)

    x = 19 - 1
    y = 1
    map.winfo_children()[20 * (x + 1) + y + 1].destroy()
    map.winfo_children()[20 * (x + 1) + y].destroy()
    map.winfo_children()[20 * (x + 1) + y - 1].destroy()
    map.winfo_children()[20 * (x) + y + 1].destroy()
    map.winfo_children()[20 * (x) + y].destroy()
    map.winfo_children()[20 * (x) + y - 1].destroy()
    map.winfo_children()[20 * (x - 1) + y + 1].destroy()
    map.winfo_children()[20 * (x - 1) + y].destroy()
    map.winfo_children()[20 * (x - 1) + y - 1].destroy()
    arrow = Canvas(map, width=86, height=86, bg="#EF5938", relief=RAISED, borderwidth=0)
    arrow.grid(row=x - 1, column=y - 1, rowspan=3, columnspan=3)
    arrow.create_text(45, 45, text=">", angle=90, anchor="w", font=('Helvetica', '50', 'bold'))

    display_scanned = Canvas(root, width=600, height=70, relief=RAISED, borderwidth=0)
    display_scanned.grid(row=21, column=0)
    display_scanned.create_text(10, 30, text='Scan:'+' '.join(scanned), anchor="w", font=('Helvetica', '10', 'bold'))



add_pillar_button = Button(root, text="Add",command=lambda: edit_pillars('add')).grid(row=1,column=1)
remove_pillar_button =  Button(root, text="Remove",command=lambda: edit_pillars('remove')).grid(row=1,column=2)

entry= Entry(root, width= 40)
entry.focus_set()
entry.grid(row=1,column=3)

run_button = Button(root, text="Run",command=lambda: run(root)).grid(row=2,column=1)

map = Frame(root,height=400,width=400,relief=SUNKEN,borderwidth=2)
map.grid(row=1,column=0,rowspan=20)

display_time = Canvas(root, width=150, height=70,relief=RAISED,borderwidth=0)
display_time.grid(row=11,column=3)
display_time.create_text(10, 30, text='Time:'+str(end-start)+'s', anchor="w", font=('Helvetica','15','bold'))

display_scanned = Canvas(root, width=600, height=70,relief=RAISED,borderwidth=0)
display_scanned.grid(row=21,column=0)
display_scanned.create_text(10, 30, text='Scan:'+' '.join(scanned), anchor="w", font=('Helvetica','10','bold'))
refresh()






root.mainloop()
