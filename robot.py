import numpy as np
import re
import random


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        #maze_map is used by the robot during the target mode
        self.maze_map = np.zeros((maze_dim,maze_dim,4),dtype=int)
        self.maze_map.fill(-1)
        
        #Create the heuristic cost table for experimentation
        #Please note that this robot does not use heuristic costs but it is implemented
        #to compare it with the other cost functions.
        self.hcost_array = np.zeros((maze_dim,maze_dim),dtype=int)
        self.goal_room =[[maze_dim/2-1,maze_dim/2-1],[maze_dim/2,maze_dim/2-1],[maze_dim/2-1,maze_dim/2],[maze_dim/2,maze_dim/2]] 
        self.goal_door = [0,0]
        for (x,y), value in np.ndenumerate(self.hcost_array):
            manhattan_dist = []
            for i in range(len(self.goal_room)):
                distance = abs(self.goal_room[i][0]-x)+abs(self.goal_room[i][1]-y)
                manhattan_dist.append(distance)
            self.hcost_array[x][y]=min(manhattan_dist)
        
        # G cost array is another cost function: The number of steps from the initial location
        self.gcost_array = np.zeros((maze_dim,maze_dim),dtype=int)
        self.gcost_array.fill(200)
        self.gcost_array[0][0] = 0
        
        #Open list contains unvisited but reachable squares
        #Closed list containes visited squares
        self.open_list =[]
        self.closed_list=[]
        self.open_list.append(self.location)
        self.closed_list.append(self.location)
        
        # Which run are we in? 
        self.run_index = 0
        #Did we print the solution?
        self.solution_printed = 0
        
        #Are we in Target or Exploration mode?
        self.mode = 'Exploration'
        
        #Which target are we tyring to reach?
        self.away_target = None
        
                
        

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        #Obtain the sensor readings and the current map
        #Obtain the heading
        heading_list = [0,1,2,3]
        heading_name_list = ['up','right','down','left']
        heading_name = self.heading
        heading = heading_name.index(self.heading)
        
        #Where are we?
        current_x = self.location[0]
        current_y = self.location[1]
        if not (self.location in self.closed_list):
            self.closed_list.append(self.location)
        if (self.location in self.open_list):
            self.open_list.remove(self.location)
     
        #Based on sensor reading update the maze map and open list and the gcost array
        self.update_maze_map(sensors)
        self.update_open_list(sensors)
        
        # Here is the strategy for run 0
        if self.run_index==0:
            
            #If the goal is found, do the necessary updates
            if (self.location in self.goal_room):
                    rotation = 'Reset'
                    movement = 'Reset'
                    self.goal_door = self.location
                    self.location = [0,0]
                    self.heading ='up'
                    self.run_index = 1
                    
            #Our strategy for run 0 is first to start exploration    
            elif re.match(self.mode,'Exploration'):
                
                #Immediate open squares are the ones we can reach with one square jumps
                immediate_open_list = self.get_immediate_open_list(sensors)
                
                if len(immediate_open_list)!=0:
                    
                    #Query the h costs of immediate targets
                    immediate_open_hcosts= []
                    for i in range(len(immediate_open_list)):
                        immediate_target_candidate = immediate_open_list[i]
                        hcost = self.hcost_array[immediate_target_candidate[0]][immediate_target_candidate[1]]
                        immediate_open_hcosts.append(hcost)
                    #Query the g costs of immediate targets
                    immediate_open_gcosts= []
                    for i in range(len(immediate_open_list)):
                        immediate_target_candidate = immediate_open_list[i]
                        gcost = self.gcost_array[immediate_target_candidate[0]][immediate_target_candidate[1]]
                        immediate_open_gcosts.append(gcost)  
                    #Calculate f costs of immediate targets
                    immediate_open_fcosts= []
                    for i in range(len(immediate_open_list)):
                        immediate_target_candidate = immediate_open_list[i]
                        fcost = immediate_open_gcosts[i] + immediate_open_hcosts[i]
                        immediate_open_fcosts.append(fcost)                          
                        
                    
                    #Decide the next move to reach the next immediate target and update location
                    immediate_target = immediate_open_list[immediate_open_gcosts.index(max(immediate_open_gcosts))]
                    rotation, movement = self.next_immediate_move(heading_name, current_x, current_y, immediate_target)
                    self.location = immediate_target
                
                # If there are not immediate open squares, we switch to Target mode:                     
                else:
                    print 'hit a dead end recalculating:'
                    #Once hit a dead end swtich to target mode to go next open square fastest way possible
                    self.mode = 'Target'
                    #Query the h costs of open squares
                    open_hcosts= []
                    for i in range(len(self.open_list)):
                        away_target_candidate = self.open_list[i]
                        hcost = self.hcost_array[away_target_candidate[0]][away_target_candidate[1]]
                        open_hcosts.append(hcost)
                    #Query the g costs of  open squares
                    open_gcosts= []
                    for i in range(len(self.open_list)):
                        away_target_candidate = self.open_list[i]
                        gcost = self.gcost_array[away_target_candidate[0]][away_target_candidate[1]]
                        open_gcosts.append(gcost)  
                    #Calculate f costs of  open squares
                    open_fcosts= []
                    for i in range(len(self.open_list)):
                        away_target_candidate = self.open_list[i]
                        fcost = open_gcosts[i] + open_hcosts[i]
                        open_fcosts.append(fcost)
                    
                    # We decided on a open square, now how do we get there? using the shortest path of course.
                    self.away_target = self.open_list[open_gcosts.index(max(open_gcosts))]
                    rotation, movement, to_away_target = self.get_shortest_path(sensors,self.away_target)
                    self.location = to_away_target
                    
                    #If we have reached the target square switch back to Exploration mode.
                    if self.location  == self.away_target:
                        self.mode='Exploration'                    
                                        
            elif re.match(self.mode,'Target'):
                    
                    #The next move to reach the away target
                    rotation, movement, to_away_target = self.get_shortest_path(sensors,self.away_target)
                    self.location = to_away_target
                    
                    #If we have reached the target square switch back to Exploration mode.
                    if self.location  == self.away_target:
                        self.mode='Exploration'                        
        
        #our startegy for run 1: Go to the goal door fastest way possible
        elif self.run_index==1:
            
            self.mode = 'Target'
            self.away_target = self.goal_door
            rotation, movement, to_away_target = self.get_shortest_path(sensors,self.away_target)
            self.location = to_away_target
    
        return rotation, movement
    
    def next_immediate_move(self,heading_name, current_x, current_y, immediate_target):
        
        #Calculate how to perform next immediate move
        x_change = immediate_target[0]-current_x
        y_change = immediate_target[1]-current_y  
        
        rotation = 0
        movement = 0
        if re.match(heading_name,'up'):
            if x_change<0:
                #perform left move
                rotation=-90
                movement=1
                self.heading = 'left'
            elif y_change>0:
                #perform forward move
                rotation=0
                movement=1
                self.heading = 'up'
            elif x_change>0:
                #perform right move
                rotation=90
                movement=1
                self.heading = 'right'
            else:
                print 'Error 0 in next immediate move definition'
                                           
        if re.match(heading_name,'down'):
            if x_change>0:
                #perform left move
                rotation=-90
                movement=1
                self.heading = 'right'
            elif y_change<0:
                #perform forward move
                rotation=0
                movement=1
                self.heading = 'down'
            elif x_change<0:
                #perform right move
                rotation=90
                movement=1
                self.heading = 'left'
            else:
                print 'Error 1 in next immediate move definition'          
               
        if re.match(heading_name,'right'):
            if y_change>0:
                #perform left move
                rotation=-90
                movement=1
                self.heading = 'up'
            elif x_change>0:
                #perform forward move
                rotation=0
                movement=1
                self.heading = 'right'
            elif y_change<0:
                #perform right move
                rotation=90
                movement=1
                self.heading = 'down'
            else:
                print 'Error 2 in next immediate move definition'           
             
        if re.match(heading_name,'left'):
            if y_change<0:
                #perform left move
                rotation=-90
                movement=1
                self.heading = 'down'
            elif x_change<0:
                #perform forward move
                rotation=0
                movement=1
                self.heading = 'left'
            elif y_change>0:
                #perform right move
                rotation=90
                movement=1
                self.heading = 'up'
            else:
                print 'Error 3 in next immediate move definition'   
        
        return rotation, movement
    
    def update_maze_map(self,sensors):
        
        #Update the maze map based on the sensor readings
        heading_name = self.heading
        left_open_count = sensors[0]
        center_open_count = sensors[1]
        right_open_count = sensors[2]        
        maze_map = self.maze_map
        current_x = self.location[0]
        current_y = self.location[1]        
    
        
        if re.match(heading_name,'up'):
            #update according to left reading
            for i in range(0,(left_open_count)):
                maze_map[current_x-i,current_y,3]=1
            for i in range(1,(left_open_count+1)):
                maze_map[current_x-i,current_y,1]=1 
            #update according to center reading
            for i in range(0,(center_open_count)):
                maze_map[current_x,current_y+i,0]=1
            for i in range(1,(center_open_count+1)):
                maze_map[current_x,current_y+i,2]=1 
            #update according to right reading
            for i in range(0,(right_open_count)):
                maze_map[current_x+i,current_y,1]=1
            for i in range(1,(right_open_count+1)):
                maze_map[current_x+i,current_y,3]=1 
        if re.match(heading_name,'down'):
            #update according to left reading
            for i in range(0,(left_open_count)):
                maze_map[current_x+i,current_y,1]=1
            for i in range(1,(left_open_count+1)):
                maze_map[current_x+i,current_y,3]=1 
            #update according to center reading
            for i in range(0,(center_open_count)):
                maze_map[current_x,current_y-i,2]=1
            for i in range(1,(center_open_count+1)):
                maze_map[current_x,current_y-i,0]=1 
            #update according to right reading
            for i in range(0,(right_open_count)):
                maze_map[current_x-i,current_y,3]=1
            for i in range(1,(right_open_count+1)):
                maze_map[current_x-i,current_y,1]=1  
        if re.match(heading_name,'right'):
            #update according to left reading
            for i in range(0,(left_open_count)):
                maze_map[current_x,current_y+i,0]=1
            for i in range(1,(left_open_count+1)):
                maze_map[current_x,current_y+i,2]=1 
            #update according to center reading
            for i in range(0,(center_open_count)):
                maze_map[current_x+i,current_y,1]=1
            for i in range(1,(center_open_count+1)):
                maze_map[current_x+i,current_y,3]=1 
            #update according to right reading
            for i in range(0,(right_open_count)):
                maze_map[current_x,current_y-i,2]=1
            for i in range(1,(right_open_count+1)):
                maze_map[current_x,current_y-i,0]=1  
        if re.match(heading_name,'left'):
            #update according to left reading
            for i in range(0,(left_open_count)):
                maze_map[current_x,current_y-i,2]=1
            for i in range(1,(left_open_count+1)):
                maze_map[current_x,current_y-i,0]=1 
            #update according to center reading
            for i in range(0,(center_open_count)):
                maze_map[current_x-i,current_y,3]=1
            for i in range(1,(center_open_count+1)):
                maze_map[current_x-i,current_y,1]=1 
            #update according to right reading
            for i in range(0,(right_open_count)):
                maze_map[current_x,current_y+i,0]=1
            for i in range(1,(right_open_count+1)):
                maze_map[current_x,current_y+i,2]=1                 
        
        self.maze_map = maze_map
        #print maze_map[:,:,2]
        return None
    
    def get_immediate_open_list(self, sensors):
        #Calculate Immediate open region for exploring
        heading_name = self.heading
        left_open_count = sensors[0]
        center_open_count = sensors[1]
        right_open_count = sensors[2]        
        maze_map = self.maze_map
        current_x = self.location[0]
        current_y = self.location[1]            
        immediate_open_list = []
        
        if re.match(heading_name,'up'):
            #update according to left reading
            if left_open_count:
                immediate_open_list.append([current_x-1,current_y])
            #update according to center reading
            if center_open_count:
                immediate_open_list.append([current_x,current_y+1])           
            #update according to right reading
            if right_open_count:
                immediate_open_list.append([current_x+1,current_y])           
        if re.match(heading_name,'down'):
            #update according to left reading
            if left_open_count:
                immediate_open_list.append([current_x+1,current_y])
            #update according to center reading
            if center_open_count:
                immediate_open_list.append([current_x,current_y-1])           
            #update according to right reading
            if right_open_count:
                immediate_open_list.append([current_x-1,current_y])
        if re.match(heading_name,'right'):
            #update according to left reading
            if left_open_count:
                immediate_open_list.append([current_x,current_y+1])
            #update according to center reading
            if center_open_count:
                immediate_open_list.append([current_x+1,current_y])           
            #update according to right reading
            if right_open_count:
                immediate_open_list.append([current_x,current_y-1])
        if re.match(heading_name,'left'):
            #update according to left reading
            if left_open_count:
                immediate_open_list.append([current_x,current_y-1])
            #update according to center reading
            if center_open_count:
                immediate_open_list.append([current_x-1,current_y])           
            #update according to right reading
            if right_open_count:
                immediate_open_list.append([current_x,current_y+1])
        
        immediate_open_list = [x for x in immediate_open_list if x not in self.closed_list]
        return immediate_open_list
    
    def update_open_list(self, sensors):
        
        #Add newly obtained open squares to open_list and update their g_cost        
        heading_name = self.heading
        left_open_count = sensors[0]
        center_open_count = sensors[1]
        right_open_count = sensors[2]        
        maze_map = self.maze_map
        current_x = self.location[0]
        current_y = self.location[1]
        current_gcost = self.gcost_array[current_x][current_y]
      
        if re.match(heading_name,'up'):
            #update according to left reading
            for i in range(1,(left_open_count+1)):
                available_next = [current_x-i,current_y]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i):
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i 
            #update according to center reading
            for i in range(1,(center_open_count+1)):
                available_next = [current_x,current_y+i]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i):
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i          
            #update according to right reading
            for i in range(1,(right_open_count+1)):
                available_next =[current_x+i,current_y] 
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i):                    
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i         
        if re.match(heading_name,'down'):
            #update according to left reading
            for i in range(1,(left_open_count+1)):
                available_next =[current_x+i,current_y] 
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
            #update according to center reading
            for i in range(1,(center_open_count+1)):
                available_next =[current_x,current_y-i] 
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i           
            #update according to right reading
            for i in range(1,(right_open_count+1)):
                available_next =[current_x-i,current_y]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
        if re.match(heading_name,'right'):
            #update according to left reading
            for i in range(1,(left_open_count+1)):
                available_next =[current_x,current_y+i]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
            #update according to center reading
            for i in range(1,(center_open_count+1)):
                available_next =[current_x+i,current_y] 
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i           
            #update according to right reading
            for i in range(1,(right_open_count+1)):
                available_next =[current_x,current_y-i] 
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
        if re.match(heading_name,'left'):
            #update according to left reading
            for i in range(1,(left_open_count+1)):
                available_next =[current_x,current_y-i]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
            #update according to center reading
            for i in range(1,(center_open_count+1)):
                available_next =[current_x-i,current_y]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i           
            #update according to right reading
            for i in range(1,(right_open_count+1)):
                available_next =[current_x,current_y+i]
                if not (( available_next in self.open_list) or  (available_next in self.closed_list)):
                    self.open_list.append(available_next)
                if self.gcost_array[available_next[0]][available_next[1]] > (current_gcost + i): 
                    self.gcost_array[available_next[0]][available_next[1]] = current_gcost + i
        return None
    
    def get_shortest_path(self,sensors,away_target):
        # This is an implementation of breadth first algorithm. Calculates the shortest path to any square
        heading_name = self.heading  
        init = self.location
        goal = away_target
        grid = self.maze_map
        
        #This parameter will be returned at the end
        to_away_target = [0,0]
        
        #The cost function to move from one square to next
        cost = 1
        delta = [[0, 1 ], # go up
                 [ 1, 0], # go right
                 [ 0, -1 ], # go down
                 [ -1, 0 ]] # go left
        delta_name = ['U', 'R', 'D', 'L']    
      
        #Open and closed lists. Please note these are not related to self.open_list and self.closed_list
        open_list =[]
        closed_list=[]
        open_list_cost = []
        current = init
        open_list.append(current[:])
        open_list_cost.append([0,0,0])
        adjacent_square=[0,0]
        current_cost=0
        cost_is_incremented = 0
        cost_list = np.zeros((len(grid), len(grid[0])), dtype=int)
        cost_list.fill(100)
        action_list=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
        
        while len(open_list):
                       
            #Stop searching if the goal has been found
            if goal==current:
                #print_path[current[0]][current[1]]='*'
                break
            
            # Get the g cost of the current square and then remove it from the open list and add it to the closed list
            closed_list.append(current)
            current_cost=open_list_cost[open_list.index(current)][0]
            del open_list_cost[open_list.index(current)]
            open_list.remove(current)
            
            #See if you can go one square away in all possible directons from the current square
            for j in range(0,len(delta)):
                #print 'j',j
                adjacent_square[0] = current[0]+delta[j][0]
                adjacent_square[1] = current[1]+delta[j][1]
                try:
                    if grid[current[0]][current[1]][j]!=-1 and not ((adjacent_square in open_list) or (adjacent_square in closed_list) or adjacent_square[0]<0  or adjacent_square[1]<0):
                        if not cost_is_incremented:
                            current_cost=current_cost+cost
                            cost_is_incremented = 1
                        open_list.append(adjacent_square[:])
                        action_list[adjacent_square[0]][adjacent_square[1]]=delta_name[j]
                        open_list_cost.append([current_cost,adjacent_square[0],adjacent_square[1]])
                        cost_list[adjacent_square[0]][adjacent_square[1]]=current_cost
                        #print 'open:',open_list
                except IndexError:
                    'sorry'
            if not len(open_list):
                path = "fail"
                break
            
            cost_is_incremented = 0
            current_cost = min(open_list_cost, key = lambda t: t[0])
            current =[current_cost[1],current_cost[2]]
  
        #Obtain the complete list of actions from the current square
        policy=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]   
        x = goal[0]
        y = goal[1]
        policy[x][y] = 'G'
        action = delta_name.index(action_list[x][y])
        x=x-delta[action][0]
        y=y-delta[action][1]
        policy[x][y]=delta_name[action]
        while x!=init[0] or y!=init[1]:
            action = delta_name.index(action_list[x][y])
            x=x-delta[action][0]
            y=y-delta[action][1]
            policy[x][y]=delta_name[action]
        
        #Print the policty to be able to see what is the solution used by the robot
        if self.run_index ==1:
            if self.solution_printed ==0:
                print 'Here is the solution of the maze:'
                policy_transpose = zip(*policy)
                for row in reversed(range(0,len(grid))):
                    print policy_transpose[row]
                    print ' '   
                self.solution_printed=1
            
        #What is the immediate action we need to take to get to the goal?
        next_absolute_action = delta[delta_name.index(policy[init[0]][init[1]])]
        to_away_target[0] = init[0] + next_absolute_action[0]
        to_away_target[1] = init[1] + next_absolute_action[1]

        # See if the next three movements are the same if so we can go in two or three jumps
        movement_multiplier  = 1
        while re.match((policy[to_away_target[0]][to_away_target[1]]),(policy[init[0]][init[1]])) and movement_multiplier <3:
            to_away_target[0] = to_away_target[0] + next_absolute_action[0]
            to_away_target[1] = to_away_target[1] + next_absolute_action[1] 
            movement_multiplier +=1
 
        rotation, movement = self.convert_absolute2relative(policy[init[0]][init[1]]) 
        movement = movement*movement_multiplier

        return rotation, movement, to_away_target
    
    def convert_absolute2relative(self,absolute_action):
        #Convert absolute actions obtained by the robot into relative actions depending on the heading
        heading_name = self.heading
        if re.match(heading_name,'up'):
            if re.match(absolute_action, 'R'):
                rotation = 90
                movement = 1
                self.heading = 'right'
            elif re.match(absolute_action, 'L'):
                rotation = -90
                movement = 1
                self.heading = 'left'
            elif re.match(absolute_action, 'U'):
                rotation = 0
                movement = 1
                self.heading = 'up'
            elif re.match(absolute_action, 'D'):
                rotation = 0
                movement = -1
                self.heading = 'up'
        if re.match(heading_name,'down'):
            if re.match(absolute_action, 'R'):
                rotation = -90
                movement = 1
                self.heading = 'right'
            elif re.match(absolute_action, 'L'):
                rotation = 90
                movement = 1                
                self.heading = 'left'
            elif re.match(absolute_action, 'U'):
                rotation = 0
                movement = -1                
                self.heading = 'down'
            elif re.match(absolute_action, 'D'):
                rotation = 0
                movement = 1
                self.heading = 'down'
        if re.match(heading_name,'right'):
            if re.match(absolute_action, 'R'):
                rotation = 0
                movement = 1
                self.heading = 'right'
            elif re.match(absolute_action, 'L'):
                rotation = 0
                movement = -1                
                self.heading = 'right'
            elif re.match(absolute_action, 'U'):
                rotation = -90
                movement = 1                
                self.heading = 'up'
            elif re.match(absolute_action, 'D'):
                rotation = 90
                movement = 1
                self.heading = 'down'
        if re.match(heading_name,'left'):
            if re.match(absolute_action, 'R'):
                rotation = 0
                movement = -1
                self.heading = 'left'
            elif re.match(absolute_action, 'L'):
                rotation = 0
                movement = 1                
                self.heading = 'left'
            elif re.match(absolute_action, 'U'):
                rotation = 90
                movement = 1                
                self.heading = 'up'
            elif re.match(absolute_action, 'D'):
                rotation = -90
                movement = 1                   
                self.heading = 'down'
        
        return rotation, movement
    
   
    
    
            
                     
                        