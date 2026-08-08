import time
import json
import collections as col
import cProfile
#constant
null = None
displacements = { '38': [-1, 0], '40': [1, 0], '37': [0, -1], '39': [0, 1] }
action_labels = { '38': 'U', '40': 'D', '37': 'L', '39': 'R' }
function_calls = 0
start_time = None
end_time = None
profiler = cProfile.Profile()

def setStartTime() :
    global start_time
    start_time = time.perf_counter()

def setEndTime() :
    global end_time
    end_time = time.perf_counter()

def getElapsedTime() :
    global end_time
    global start_time
    return end_time - start_time

def incrementAndPrint() :
    global function_calls
    function_calls += 1
    print('number of calls : ' + str(function_calls))


def main() :

    user_input = input('input level : \n')

    pushworld_level = convertDataToUseable(user_input)
    dictionary = {}

    profiler.enable()
    [solutions, visited_state_list] = BFS(pushworld_level)
    profiler.disable()
    for array in solutions :
        print(array)
    print(len(visited_state_list))

    profiler.print_stats()

    print(f"elapsed time : {getElapsedTime():.2f} seconds")

    return 0



def convertDataToUseable(stringified) :
    modified_data = json.loads(stringified)
    modified_data['goalObjects'] = []
    for key_name in modified_data :
        for obj_index in range(len(modified_data[key_name])) : #returns lists of dicts
            if 'boundaryPixels' in modified_data[key_name][obj_index] :
                if key_name != 'walls' :    
                    modified_data[key_name][obj_index]['boundaryPixels'] = { tuple(pixel) for pixel in modified_data[key_name][obj_index]['boundaryPixels'] }
                    if 'goal_position' in modified_data[key_name][obj_index] : 
                        modified_data[key_name][obj_index]['goal_position'] = tuple(modified_data[key_name][obj_index]['goal_position'])
                        modified_data['goalObjects'].append((obj_index, modified_data[key_name][obj_index]['goal_position'])) #stores index and the goal position

                else :
                    modified_data[key_name][obj_index]['boundaryPixels'] = { tuple(addPoints(modified_data[key_name][obj_index]['position'], pixel)) for pixel in modified_data[key_name][obj_index]['boundaryPixels'] }
    modified_data['goalObjects'] = tuple(modified_data['goalObjects'])

    if not (modified_data['walls'][0]['id'] == 'w') :
        temp_array = [modified_data['walls'][1], modified_data['walls'][0]] #swap
        modified_data['walls'] = temp_array

    return modified_data

def BFS(pushworld) : #already sorted file pushworld
    transitive_stopping_amount = 0
    generated_duplicated_states = 0 #different paths leading to same state
    print('starting BFS...')
    state_list = col.deque() #index 0 is the state, index 1 is the steps
    state_list.appendleft([tuple(tuple(moveable['position']) for moveable in pushworld['moveables']), ''])
    visited_state_set = set()
    solutions = []
    condition = True

    id_to_index = { pushworld['moveables'][index]['id'] : index for index in range(len(pushworld['moveables'])) }

    setStartTime()


    while len(state_list) > 0 and condition :

        state = state_list.pop() #each state has length of 2
                                # 0 for the state, and 1 for the steps to get to that state
        if state[0] in visited_state_set : 
            continue
        visited_state_set.add(state[0])

        for key in displacements : 
                                                    #addPoints that returns tuples in long form
            absolute_moveables_pixels = ([{(pixel[0] + state[0][index][0], pixel[1] + state[0][index][1]) for pixel in pushworld['moveables'][index]['boundaryPixels']} 
                                          for index in range(len(state[0]))]) #add a fixed absolute moveables for the next 4 state generated
            
            [next_state, transitive_stopping] = simulateStep(pushworld, state[0], 
                                                            displacements[key], absolute_moveables_pixels, id_to_index)

            if transitive_stopping : #if not stopped and not visited yet
                transitive_stopping_amount += 1
                continue

            if next_state in visited_state_set : #to prevent an already visited state from beign added to the list
                generated_duplicated_states += 1
                continue

            temp = [next_state, state[1] + action_labels[key]]

            is_solved = True

            for goal_position in pushworld['goalObjects'] : #checks if it's solved
                if not (next_state[goal_position[0]] == goal_position[1]) : #goal_position[0] is the position in state
                    is_solved = False                                   #goal_position[1] is the target

            if is_solved : #checks if it's solved

                solutions.append(temp)

            else :

                state_list.appendleft(temp)

    setEndTime()
    print('number of generated states but different paths : ' + str(generated_duplicated_states))
    print('number of illegal moves from simulateStep : ' + str(transitive_stopping_amount))
    return [solutions, visited_state_set]

def DFS(pushworld) : 
    pass

def simulateStep(pushworld, state, displacement, absolute_moveables_pixels, id_to_index) : #pushworld = dict
    [pushed_object_ids, transitive_stopping] = getPushedObjects(pushworld, state, displacement, absolute_moveables_pixels, id_to_index) #state tuples
    if not transitive_stopping :
        next_state = []

        #mine

        #mine 

        for i in range(len(state)) :
            obj = pushworld['moveables'][i]
            pos = state[i]

            if obj['id'] in pushed_object_ids :
                next_state.append((displacement[0] + pos[0], displacement[1] + pos[1]))
            else :
                next_state.append(pos)
        next_state = tuple(next_state)
    else :
        next_state = state

    return [next_state, transitive_stopping]

def getPushedObjects(pushworld, state, displacement, absolute_moveables_pixels, id_to_index) :

    actor = pushworld['moveables'][0]
    pushed_object_ids = []
    transitive_stopping = False
    moving_parts = [actor]
    absolute_fixed_walls = pushworld['walls'][0]['boundaryPixels'] #index 0 is walls in the lord we trust #tuple
    absolute_actor_walls = [] if len(pushworld['walls']) < 2 else pushworld['walls'][1]['boundaryPixels']
    
    while len(moving_parts) > 0 and not transitive_stopping :
        obj = moving_parts.pop()

        if (obj['id'] in pushed_object_ids) : continue

        pushed_object_ids.append(obj['id'])

        for boundaryPixel in absolute_moveables_pixels[id_to_index[obj['id']]] :
            after_displacement = (boundaryPixel[0] + displacement[0], boundaryPixel[1] + displacement[1]) #adds 2 vector? coordinate?
            if after_displacement in absolute_fixed_walls :                                             # and then make it tuple
                transitive_stopping = True
                break

            for index in range(len(absolute_moveables_pixels)) :
                if index == id_to_index[obj['id']] : continue #ignores self pixel hitbox
                if after_displacement in absolute_moveables_pixels[index] : #checks if pixels overlap         
                    moving_parts.append(pushworld['moveables'][index])  

            if obj['id'] == 'a' :
                if after_displacement in absolute_actor_walls : 
                    transitive_stopping = True
                    break

    return [pushed_object_ids, transitive_stopping]

def isGoalState(pushworld, state) :
    is_solved = False
    for i in range(len(state)) :
        moveable = pushworld['moveables'][i]
        if 'goal_position' in moveable : #if it a goal object
            pos = state[i]
            if moveable['goal_position'] == pos : #if the goal_position and goal moveable object is on the same point
                is_solved = True
            else :
                is_solved = False
                break

    return is_solved

def addPoints(p1, p2) :
    return [p1[0] + p2[0], p1[1] + p2[1]]

def subPoints(p1, p2) :
    return [p1[0] - p2[0], p1[1] - p2[1]]


def addPointsTuple(p1, p2) :
    return (p1[0] + p2[0], p1[1] + p2[1])

def subPointsTuple(p1, p2) :
    return (p1[0] - p2[0], p1[1] - p2[1])


def is2DPointInArray(p, array) : #unused
    if p in array : return True
    return False


def getObjectIDsToPositions(pushworld, state) :
    id_to_pos = {}

    for w in pushworld['walls'] :
        id_to_pos[w['id']] = w['position']

    for g in pushworld['goals'] : 
        id_to_pos[g['id']] = g['position']


    for i in range(len(state)) :
        id_to_pos[pushworld['moveables'][i]['id']] = state[i]

    return id_to_pos




def get2DMin(pixels) :

    if len(pixels) == 1 :
        return pixels[0]
    min_x = pixels[0][0]
    min_y = pixels[0][1]
    for i in pixels :
        if i[0] < min_x :
            min_x = i[0]
        if i[1] < min_y :
            min_y = i[1]

    return [min_x, min_y]

main()

