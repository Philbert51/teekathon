# region Libraries
import time
import json
import collections as col
import cProfile
import subprocess
import pathlib
import google.genai as genai
import typing
import dotenv
import traceback
import psutil
import threading
import uuid
import tempfile
import sys
import queue as q
from os import environ
from flask import Flask, request, Response
from flask_cors import CORS
dotenv.load_dotenv(dotenv.find_dotenv())
# endregion

# region Initialize
null = None
models = typing.Literal["gemini-2.5-flash", "gemini-2.5-pro", "gemini-2.5-flash-preview-tts", "gemini-2.5-pro-preview-tts", "gemma-4-26b-a4b-it", "gemma-4-31b-it", "gemini-flash-latest", "gemini-flash-lite-latest", "gemini-pro-latest", "gemini-2.5-flash-lite", "gemini-2.5-flash-image", "gemini-3-flash-preview", "gemini-3.1-pro-preview", "gemini-3.1-pro-preview-customtools", "gemini-3.1-flash-lite-preview", "gemini-3.1-flash-lite", "gemini-3-pro-image-preview", "gemini-3-pro-image", "nano-banana-pro-preview", "gemini-3.1-flash-image-preview", "gemini-3.1-flash-image", "gemini-3.1-flash-lite-image", "gemini-3.5-flash", "gemini-3.5-flash-lite", "gemini-omni-flash-preview", "gemini-3.6-flash", "gemini-3.7-flash", "lyria-3-clip-preview", "lyria-3-pro-preview", "gemini-3.1-flash-tts-preview", "gemini-robotics-er-1.6-preview", "gemini-robotics-er-2-preview", "gemini-2.5-computer-use-preview-10-2025", "antigravity-preview-05-2026", "deep-research-max-preview-04-2026", "deep-research-preview-04-2026", "deep-research-pro-preview-12-2025", "gemini-embedding-001", "gemini-embedding-2-preview", "gemini-embedding-2", "aqa", "imagen-4.0-generate-001", "imagen-4.0-ultra-generate-001", "imagen-4.0-fast-generate-001", "veo-3.1-generate-preview", "veo-3.1-fast-generate-preview", "veo-3.1-lite-generate-preview", "gemini-2.5-flash-native-audio-latest", "gemini-2.5-flash-native-audio-preview-09-2025", "gemini-2.5-flash-native-audio-preview-12-2025", "gemini-3.1-flash-live-preview", "gemini-robotics-er-2-streaming-preview", "gemini-3.5-live-translate-preview"]
displacements = { '38': [-1, 0], '40': [1, 0], '37': [0, -1], '39': [0, 1] }
action_labels = { '38': 'U', '40': 'D', '37': 'L', '39': 'R' }
action_label_to_vector = { 'U' : [-1, 0], 'D' : [1, 0], 'L' : [0, -1], 'R' : [0, 1] }
function_calls = 0
profiler = cProfile.Profile()
GOOGLE_GENAI_API_KEY = environ.get('GOOGLE_GENAI_API_KEY')
google_genai_client = genai.Client(api_key=GOOGLE_GENAI_API_KEY)
main_models : list[models] = ['gemini-3.1-flash-lite', 'gemini-3.5-flash','gemini-3.5-flash-lite', 'gemini-3.6-flash', 'gemini-3.7-flash','gemini-2.5-flash-lite','gemini-2.5-flash']
port : int = int(environ.get('PORT', 8000))
host = '0.0.0.0'
allowed_origins : list[str] = ["http://127.0.0.1", 'http://localhost']
environ_origins = environ.get("ALLOWED_ORIGINS")
if not (environ_origins is None) :
    for value in environ_origins.split(',') :
        allowed_origins.append(value)

list_process : dict = {}
input_test = [{"moveables":[{"id":"a","boundaryPixels":[[0,0]],"position":[9,3]},{"id":"m0","boundaryPixels":[[0,0],[0,1],[1,0],[1,1]],"position":[4,3],"goal_position":[6,6]},{"id":"m1","boundaryPixels":[[0,0],[0,1],[0,2],[0,3],[0,4],[1,4],[2,0],[2,1],[2,2],[2,3],[2,4],[3,0],[4,0],[4,1],[4,2],[4,3],[4,4]],"position":[7,7]}],"walls":[{"id":"aw","boundaryPixels":[[0,0],[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],[0,8],[0,9],[1,0],[1,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7],[1,8],[1,9]],"position":[1,2]},{"id":"w","boundaryPixels":[[0,0],[0,12],[1,0],[1,12],[2,0],[2,12],[3,0],[3,12],[4,0],[4,12],[5,0],[5,12],[6,0],[6,12],[7,0],[7,12],[8,0],[8,12],[9,0],[9,12],[10,0],[10,12],[11,0],[11,12],[12,0],[12,12],[0,1],[12,1],[0,2],[12,2],[0,3],[12,3],[0,4],[12,4],[0,5],[12,5],[0,6],[12,6],[0,7],[12,7],[0,8],[12,8],[0,9],[12,9],[0,10],[12,10],[0,11],[12,11]],"position":[0,0]}],"goals":[{"id":"g0","boundaryPixels":[[0,0],[0,1],[1,0],[1,1]],"position":[6,6]}]},
              {"moveables":[{"id":"a","boundaryPixels":[[0,0],[0,1],[0,2],[1,1]],"position":[6,4]},{"id":"m0","boundaryPixels":[[0,0],[1,0],[2,0],[2,1]],"position":[2,1],"goal_position":[8,1]},{"id":"m1","boundaryPixels":[[0,0]],"position":[2,7],"goal_position":[6,9]}],"walls":[{"id":"w","boundaryPixels":[[5,8],[5,9],[7,8],[7,9],[0,0],[0,10],[1,0],[1,10],[2,0],[2,10],[3,0],[3,10],[4,0],[4,10],[5,0],[5,10],[6,0],[6,10],[7,0],[7,10],[8,0],[8,10],[9,0],[9,10],[10,0],[10,10],[11,0],[11,10],[0,1],[11,1],[0,2],[11,2],[0,3],[11,3],[0,4],[11,4],[0,5],[11,5],[0,6],[11,6],[0,7],[11,7],[0,8],[11,8],[0,9],[11,9]],"position":[0,0]}],"goals":[{"id":"g0","boundaryPixels":[[0,0],[1,0],[2,0],[2,1]],"position":[8,1]},{"id":"g1","boundaryPixels":[[0,0]],"position":[6,9]}]},
              {"moveables":[{"id":"a","boundaryPixels":[[0,0]],"position":[4,2]},{"id":"m0","boundaryPixels":[[0,0]],"position":[4,7],"goal_position":[4,7]},{"id":"m1","boundaryPixels":[[0,0],[0,1],[1,0],[1,1],[2,0],[2,1],[3,0],[3,1],[4,0],[4,1]],"position":[3,4],"goal_position":[3,12]}],"walls":[{"id":"w","boundaryPixels":[[0,0],[0,14],[1,0],[1,14],[2,0],[2,14],[3,0],[3,14],[4,0],[4,14],[5,0],[5,14],[6,0],[6,14],[7,0],[7,14],[8,0],[8,14],[9,0],[9,14],[0,1],[9,1],[0,2],[9,2],[0,3],[9,3],[0,4],[9,4],[0,5],[9,5],[0,6],[9,6],[0,7],[9,7],[0,8],[9,8],[0,9],[9,9],[0,10],[9,10],[0,11],[9,11],[0,12],[9,12],[0,13],[9,13]],"position":[0,0]}],"goals":[{"id":"g0","boundaryPixels":[[0,0]],"position":[4,7]},{"id":"g1","boundaryPixels":[[0,0],[0,1],[1,0],[1,1],[2,0],[2,1],[3,0],[3,1],[4,0],[4,1]],"position":[3,12]}]}]
prompt = """You are writing a Python program that solves a PushWorld puzzle.

PushWorld is a grid puzzle. An agent moves one cell at a time in four directions.
When the agent moves into a movable object, it pushes it. Pushed objects can push
other objects in turn. A push is blocked entirely if any object in the chain would
move into a wall. Walls never move. Goals are not solid and do not block anything.
The puzzle is solved when every movable that has a goal_position is standing on it.

The puzzle is a dict with these keys:

  moveables : list of objects. Index 0 is always the agent.
              Each has 'id' (str), 'position' ([row, col] anchor),
              and 'boundaryPixels' (list of [row, col] offsets from the anchor).
              Some also have 'goal_position' ([row, col]) - these must reach it.
  walls     : list of wall objects, same shape. 'w' blocks everything.
              'aw' blocks only the agent.
  goals     : list of goal markers. Decorative - ignore them.

  Every puzzle is solvable. There is always at least one movable with a goal_position. The walls list always contains 'w' and may or may not contain 'aw'.

Actions are the single characters 'U', 'D', 'L', 'R'.
'U' is row - 1, 'D' is row + 1, 'L' is col - 1, 'R' is col + 1.

Write a function with exactly this signature:

    def solve(puzzle):
        ...
        return "UDLR..."

        

It takes the puzzle dict and returns a string of actions that solves it.
Return an empty string if you find no solution.

<example_input> {"moveables":[{"id":"a","boundaryPixels":[[0,0]],"position":[9,3]},{"id":"m0","boundaryPixels":[[0,0],[0,1],[1,0],[1,1]],"position":[4,3],"goal_position":[6,6]},{"id":"m1","boundaryPixels":[[0,0],[0,1],[0,2],[0,3],[0,4],[1,4],[2,0],[2,1],[2,2],[2,3],[2,4],[3,0],[4,0],[4,1],[4,2],[4,3],[4,4]],"position":[7,7]}],"walls":[{"id":"aw","boundaryPixels":[[0,0],[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],[0,8],[0,9],[1,0],[1,1],[1,2],[1,3],[1,4],[1,5],[1,6],[1,7],[1,8],[1,9]],"position":[1,2]},{"id":"w","boundaryPixels":[[0,0],[0,12],[1,0],[1,12],[2,0],[2,12],[3,0],[3,12],[4,0],[4,12],[5,0],[5,12],[6,0],[6,12],[7,0],[7,12],[8,0],[8,12],[9,0],[9,12],[10,0],[10,12],[11,0],[11,12],[12,0],[12,12],[0,1],[12,1],[0,2],[12,2],[0,3],[12,3],[0,4],[12,4],[0,5],[12,5],[0,6],[12,6],[0,7],[12,7],[0,8],[12,8],[0,9],[12,9],[0,10],[12,10],[0,11],[12,11]],"position":[0,0]}],"goals":[{"id":"g0","boundaryPixels":[[0,0],[0,1],[1,0],[1,1]],"position":[6,6]}]} </example_input>

Requirements:
- Use only the Python standard library.
- Do not read files, access the network, or read environment variables.
- Do not hardcode solutions, lookup tables, or branch on specific puzzle contents.
  The same program will be run unchanged on puzzles you have not seen.
- Write a general search. Shorter and more general programs are preferred.

Output only the Python code. No explanation, no markdown fences.
"""

done_message = object()
# current_window = ctypes.windll.kernel32.GetConsoleWindow()
# endregion

def incrementAndPrint() :
    global function_calls
    function_calls += 1
    print('number of calls : ' + str(function_calls))

def updateMessageQueue(message, queue : q.Queue) :
    print(message)
    if queue is not None :
        queue.put(message)

def refreshListProcess() :
    for key in list_process :
        pass

def curriculum(queue : q.Queue) :
    updateMessageQueue("Starting LLM, Curriculum Mode = True...", queue)
    updateMessageQueue('This may take a while..', queue)

    previous_interaction_id = None
    current_prompt = prompt
    version = 0

    for index in range(len(input_test)) :
        updateMessageQueue(f"Stage {index + 1} of {len(input_test)}", queue)
        stage_solved = False

        for attempt in range(3) :

            output = createGenAIPrompt(prompt = current_prompt, previous_interaction_id = previous_interaction_id, queue = queue)
            if not output :
                updateMessageQueue("No output from LLM, stopping", queue)
                return
            previous_interaction_id = output.id
            version += 1
            updateMessageQueue(f"Program version {version} :\n{output.output_text}", queue)

            updateMessageQueue(f"Attempt {attempt + 1} : testing against all {index + 1} puzzle(s) so far..", queue)

            feedback = None
            #curriculum re-tests every puzzle up to and including this stage,
            #so a fix for the new puzzle cannot silently break an earlier one
            for test_index in range(index + 1) :

                temp = tempfile.NamedTemporaryFile(suffix = '.py', delete = False, mode = 'w')
                temp.write(output.output_text)
                temp.write("\n\nimport json\n")
                temp.write(f"print(solve(json.loads({json.dumps(json.dumps(input_test[test_index]))})))\n")
                temp.close()

                try :
                    sp_output = subprocess.run([sys.executable, temp.name],
                                                capture_output = True,
                                                text = True,
                                                timeout = 30,
                                                env = {})
                except subprocess.TimeoutExpired :
                    feedback = f"puzzle {test_index + 1} : timeout, program exceeded 30 seconds"
                    break

                if sp_output.returncode != 0 :
                    feedback = f"puzzle {test_index + 1} : {sp_output.stderr}"
                    break

                moves = sp_output.stdout.strip()
                solved, reason = validateSolution(input_test[test_index], moves, queue)
                if not solved :
                    regressed = test_index < index
                    label = "regression on previously solved" if regressed else "failed on"
                    feedback = f"{label} puzzle {test_index + 1} : {reason}"
                    break

            #no error
            if feedback is None :
                updateMessageQueue(f"Stage {index + 1} passed on attempt {attempt + 1}", queue)
                stage_solved = True
                break

            updateMessageQueue(f"Attempt {attempt + 1} failed : {feedback}", queue)
            current_prompt = f"Your previous program failed.\n\nFailure : {feedback}\n\nThe same program must solve every puzzle it has already solved plus the new one. Fix it and output the corrected program. Output only the Python code."

        if not stage_solved :
            updateMessageQueue(f"Gave up at stage {index + 1} after 3 attempts", queue)
            return

    updateMessageQueue("Curriculum complete : one program solves every given test puzzle", queue)
    updateMessageQueue(f"Final program : \n{output.output_text}", queue)

def nonCurriculum(queue : q.Queue) :
    updateMessageQueue("Starting LLM, Curriculum Mode = False...", queue)

    previous_interaction_id = None
    current_prompt = prompt
    puzzle = input_test[0]
    feedback = None
    version = 0

    #tries 3 times
    for attempt in range(3) :

        output = createGenAIPrompt(prompt = current_prompt, previous_interaction_id = previous_interaction_id, queue = queue)
        if not output :
            updateMessageQueue("No output from LLM, stopping", queue)
            return
        previous_interaction_id = output.id
        version += 1
        updateMessageQueue(f"Program version {version} :\n{output.output_text}", queue)

        #write the generated program plus a driver that runs it on the puzzle
        temp = tempfile.NamedTemporaryFile(suffix = '.py', delete = False, mode = 'w')
        temp.write(output.output_text)
        temp.write("\n\nimport json\n")
        temp.write(f"print(solve(json.loads({json.dumps(json.dumps(puzzle))})))\n")
        temp.close()

        updateMessageQueue(f"Attempt {attempt + 1} : running and testing generated program with a given puzzle..", queue)

        try :
            sp_output = subprocess.run([sys.executable, temp.name],
                                       capture_output = True, 
                                       text = True,
                                       timeout = 30, 
                                       env = {})
        except subprocess.TimeoutExpired :
            feedback = "timeout : program exceeded 30 seconds"
        else :
            if sp_output.returncode != 0 :
                feedback = sp_output.stderr
            else :
                moves = sp_output.stdout.strip()
                updateMessageQueue(f"Program returned : {moves}", queue)
                solved, reason = validateSolution(puzzle, moves, queue)
                feedback = None if solved else reason

        #no error
        if feedback is None :
            updateMessageQueue(f"Solved on attempt {attempt + 1}", queue)
            updateMessageQueue(f"Python script : \n{output.output_text}", queue)
            return

        updateMessageQueue(f"Attempt {attempt + 1} failed : {feedback}", queue)
        current_prompt = f"Your previous program failed.\n\nFailure : {feedback}\n\nFix it and output the corrected program. Output only the Python code."

    if feedback :
        updateMessageQueue("last program failure reason : " + feedback, queue)
    updateMessageQueue("Gave up after 3 attempts", queue)

def main() :
    #if not imported
    global host, port
    if __name__ == '__main__' :
        server = Flask(__name__)
        CORS(server, origins = allowed_origins)
        #region Handlers
        @server.route('/', methods = ['POST', 'GET'])
        def handler() :
            print('Received data')
            print(request.headers.get('Origin'))
            print('Method :', request.method)
            status = 'Success', 200
            if request.method == 'POST' :
                data = request.get_json()
                print(data)
                if (data['algorithm'] in ('BFS', 'DFS', 'RGD', 'LLM')) :
                    run_id = str(uuid.uuid4())
                    data['created_timestamp'] = time.time()
                    list_process[run_id] = data
                    status = {'id' : run_id}, 200
                else :
                    status = "Error : unknown algorithm. Must be 'BFS', 'DFS', 'LLM', or 'RGD'", 400
            return status

        @server.route('/receive/<run_id>')
        def stream(run_id) :
            data = list_process.get(run_id)
            if not data :
                return 'Error : invalid id', 404

            queue = q.Queue()

            if data['algorithm'] == 'BFS' :
                target, kwargs = BFS, {'pushworld' : convertDataToUseable(data['puzzle']), 'queue' : queue}

            elif data['algorithm'] == 'DFS' :
                target, kwargs = DFS, {'pushworld' : convertDataToUseable(data['puzzle']), 'queue' : queue}

            elif data['algorithm'] == 'RGD' :
                temp = tempfile.NamedTemporaryFile(suffix = '.pwp', delete = False, mode = 'w')
                temp.write(data['puzzle'])
                temp.close()
                target, kwargs = RGD, {'mode' : 'RGD', 'puzzle_path' : temp.name, 'queue' : queue}

            elif data['algorithm'] == 'LLM' :
                if data['curriculum'] :
                    # target, kwargs = 
                    target, kwargs = curriculum, {'queue' : queue}
                else :
                    target, kwargs = nonCurriculum, {'queue' : queue}

            #the solver runs on its own thread so the generator below can yield
            #messages while it is still working
            def run() :
                start_time = time.perf_counter()
                try :
                    target(**kwargs)
                except Exception :
                    traceback.print_exc()
                    queue.put('Error : ' + traceback.format_exc())
                finally :
                    end_time = time.perf_counter()
                    updateMessageQueue(f"Elapsed time : {end_time - start_time}s", queue)
                    queue.put(done_message)
                    

            threading.Thread(target = run, daemon = True).start()

            def inner_generator() :
                try :
                    while True :
                        try :
                            message = queue.get(timeout = 1)
                        except q.Empty :
                            yield ': keepalive\n\n'  #stops proxies dropping an idle connection
                            continue
                        if message is done_message :
                            break
                        payload = ''.join(f"data: {line}\n" for line in str(message).split('\n'))
                        yield payload + '\n'
                finally :
                    list_process.pop(run_id, None)

            return Response(inner_generator(), mimetype = 'text/event-stream')
        #endregion
        freePort(host, port)
        server.run(host, port)


def freePort(host, port) :
    """
    Terminates whatever process is currently listening on the given host and port.
    Does nothing if the port is already free.

    Passing '0.0.0.0' as the host matches a listener on any local address,
    since a wildcard bind conflicts with all of them.
    """

    terminated = 0

    for connection in psutil.net_connections(kind = 'inet') :

        #only listening sockets hold a port, established ones do not
        if connection.status != psutil.CONN_LISTEN :
            continue

        #wrong port, not our problem
        if connection.laddr.port != port :
            continue

        #a specific host only conflicts with a listener on that same address,
        #while the wildcard conflicts with every address on this port
        if host != '0.0.0.0' and connection.laddr.ip != host :
            continue

        #the owning process is not always visible without elevated privileges
        if connection.pid is None :
            continue

        process = psutil.Process(connection.pid)
        print(f"Port {port} held by {process.name()} (PID {connection.pid}) : terminating")
        process.terminate()
        terminated += 1

    if terminated == 0 :
        print(f"Port {port} is free")

    print("Number of terminated socket :", terminated)

def validateSolution(puzzle_dict, solution, queue = None) :
    """
    Replays a solution string against the simulator.
    Returns (True, None) if it solves the puzzle,
    (False, reason) otherwise.
    """
    pushworld = convertDataToUseable(json.dumps(puzzle_dict))
    state = tuple(tuple(moveable['position']) for moveable in pushworld['moveables'])
    id_to_index = { pushworld['moveables'][index]['id'] : index for index in range(len(state)) }

    for step, char in enumerate(solution) :

        if char not in action_label_to_vector :
            return False, f"invalid action '{char}' at step {step}"

        absolute_moveables_pixels = [
            {(pixel[0] + state[index][0], pixel[1] + state[index][1])
             for pixel in pushworld['moveables'][index]['boundaryPixels']}
            for index in range(len(state))
        ]

        [next_state, transitive_stopping] = simulateStep(
            pushworld, state,
            action_label_to_vector[char],
            absolute_moveables_pixels, id_to_index
        )

        if transitive_stopping :
            return False, f"illegal move '{char}' at step {step} : push was blocked"

        state = tuple(tuple(entry) for entry in next_state)

    for goal in pushworld['goalObjects'] :
        if state[goal[0]] != goal[1] :
            return False, f"finished all {len(solution)} moves but goal objects are not in their designated goal position"

    return True, None


#this function tries to make an API call with the required parameter
def createGenAIPrompt(prompt : str = None, previous_interaction_id : str = None, queue : q.Queue = None) :
    updateMessageQueue("Sending API request...", queue)
    global google_genai_client
    output = None
    if not google_genai_client :
        google_genai_client = genai.Client(api_key=GOOGLE_GENAI_API_KEY)
    for model in main_models :
        try :
            updateMessageQueue(f"Trying with {model} : ", queue)
            output = google_genai_client.interactions.create(
                model = model,
                input = prompt,
                generation_config = {'thinking_summaries' : 'auto'},
                previous_interaction_id = previous_interaction_id
            )
            while (output.status == 'in_progress') :
                time.sleep(1)
                updateMessageQueue('fetching progress : ', queue)
                output = google_genai_client.interactions.get(output.id)
                updateMessageQueue("status : " + output.status, queue)
        except Exception as e :
            traceback.print_exc()
            #tries to check with agent and try request with agent instead
            if 'refers to ' in str(e) :
                updateMessageQueue("Request failed : Trying with agent instead. \nThis will take a while", queue)
                output = google_genai_client.interactions.create(
                        agent = model,
                        input = prompt,
                        background=True,
                        previous_interaction_id = previous_interaction_id
                    )
                while (output.status == 'in_progress') :

                    time.sleep(1)
                    updateMessageQueue('fetching progress : ', queue)
                    output = google_genai_client.interactions.get(output.id)
                    updateMessageQueue("status : " + output.status, queue)
    
        if output is not None and output.status == 'completed' :
            break
        else :
            updateMessageQueue("LLM Request failed. ", queue)
        

    if not(output is None or output.errors):
        updateMessageQueue("Completed request : no errors", queue)
    else :
        updateMessageQueue("Completed request with errors", queue)
        if output is None :
            updateMessageQueue("Received no output", queue)
        else :
            for error in output.errors :
                updateMessageQueue(error, queue)
    return output

def BFS(pushworld, queue : q.Queue) : #already sorted file pushworld
    transitive_stopping_amount = 0
    generated_duplicated_states = 0 #different paths leading to same state
    updateMessageQueue(f"starting BFS...", queue)
    state_list = col.deque() #index 0 is the state, index 1 is the steps
    state_list.appendleft([tuple(tuple(moveable['position']) for moveable in pushworld['moveables']), ''])
    visited_state_set = set()
    solutions = []
    condition = True
    limit = time.perf_counter() + 30
    id_to_index = { pushworld['moveables'][index]['id'] : index for index in range(len(pushworld['moveables'])) }



    while len(state_list) > 0 and condition :

        state = state_list.pop() #each state has length of 2
                                # 0 for the state, and 1 for the steps to get to that state
        if state[0] in visited_state_set : 
            continue
        visited_state_set.add(state[0])
        if time.perf_counter() > limit :
            updateMessageQueue('Process exceeded 30 seconds : terminating BFS...', queue)
            return

        for key in displacements : 
                                                    #addPoints that returns tuples in long form
            absolute_moveables_pixels = ([{(pixel[0] + state[0][index][0], pixel[1] + state[0][index][1]) for pixel in pushworld['moveables'][index]['boundaryPixels']} 
                                          for index in range(len(state[0]))]) #add a fixed absolute moveables for the next 4 state generated
            
            [next_state, transitive_stopping] = simulateStep(pushworld, state[0], 
                                                            displacements[key], absolute_moveables_pixels, id_to_index)

            if transitive_stopping : #if not stopped
                transitive_stopping_amount += 1
                continue
 
            #to prevent an already visited state from beign added to the list
            if next_state in visited_state_set : 
                generated_duplicated_states += 1
                continue

            temp = (next_state, state[1] + action_labels[key])

            is_solved : bool = True

            for goal_position in pushworld['goalObjects'] : #checks if it's solved
                if not (next_state[goal_position[0]] == goal_position[1]) : #goal_position[0] is the position in state
                    is_solved = False                                   #goal_position[1] is the target

            if is_solved : #checks if it's solved

                solutions.append(temp)
                condition = False
                break

            else :

                state_list.appendleft(temp)


    updateMessageQueue('number of generated states but different paths : ' + str(generated_duplicated_states), queue)
    updateMessageQueue('number of illegal moves from simulateStep : ' + str(transitive_stopping_amount), queue)
    updateMessageQueue(f"Solution : {solutions[0][1]}", queue)
    updateMessageQueue(f"number of unique visited state : {len(visited_state_set)}", queue)
    return [solutions, visited_state_set]

def DFS(pushworld, queue : q.Queue) : 
    #bfs but change appendleft to append
    transitive_stopping_amount = 0
    generated_duplicated_states = 0 #different paths leading to same state
    updateMessageQueue(f"starting DFS...", queue)
    state_list = col.deque() #index 0 is the state, index 1 is the steps
    state_list.append([tuple(tuple(moveable['position']) for moveable in pushworld['moveables']), ''])
    visited_state_set = set()
    solutions = []
    condition = True
    limit = time.perf_counter() + 30

    id_to_index = { pushworld['moveables'][index]['id'] : index for index in range(len(pushworld['moveables'])) }


    while len(state_list) > 0 and condition :

        state = state_list.pop() #each state has length of 2
                                # 0 for the state, and 1 for the steps to get to that state
        if state[0] in visited_state_set : 
            continue
        visited_state_set.add(state[0])
        if time.perf_counter() > limit :
            updateMessageQueue('Process exceeded 30 seconds : terminating DFS...', queue)
            return

        for key in displacements : 
                                                    #addPoints that returns tuples in long form
            absolute_moveables_pixels = ([{(pixel[0] + state[0][index][0], pixel[1] + state[0][index][1]) for pixel in pushworld['moveables'][index]['boundaryPixels']} 
                                        for index in range(len(state[0]))]) #add a fixed absolute moveables for the next 4 state generated
            
            [next_state, transitive_stopping] = simulateStep(pushworld, state[0], 
                                                            displacements[key], absolute_moveables_pixels, id_to_index)

            if transitive_stopping : #if not stopped
                transitive_stopping_amount += 1
                continue

            #to prevent an already visited state from beign added to the list
            if next_state in visited_state_set : 
                generated_duplicated_states += 1
                continue

            temp = (next_state, state[1] + action_labels[key])

            is_solved : bool = True

            for goal_position in pushworld['goalObjects'] : #checks if it's solved
                if not (next_state[goal_position[0]] == goal_position[1]) : #goal_position[0] is the position in state
                    is_solved = False                                   #goal_position[1] is the target

            if is_solved : #checks if it's solved

                solutions.append(temp)
                condition = False
                break

            else :

                state_list.append(temp)


    updateMessageQueue('number of generated states but different paths : ' + str(generated_duplicated_states), queue)
    updateMessageQueue('number of illegal moves from simulateStep : ' + str(transitive_stopping_amount), queue)
    updateMessageQueue(f"Solution : {solutions[0][1]}", queue)
    updateMessageQueue(f"number of unique visited state : {len(visited_state_set)}", queue)
    return [solutions, visited_state_set]

def RGD(planner_path = None, mode : typing.Literal['RGD', 'N+RGD']= 'RGD', puzzle_path = None, queue : q.Queue = None) : 
    updateMessageQueue('Starting RGD...', queue)
    if (isinstance(mode, int)) :
        if mode == 0 :
            mode = 'RGD'
        elif mode == 1 :
            mode = 'N+RGD'
        else :
            raise ValueError("Valid range 0, 1 : 0 = RGD, 1 = N+RGD.")
    elif (isinstance(mode, str)) :
        if not (mode == 'RGD' or mode == 'N+RGD') :
            raise ValueError("Valid enum : RGD, N+RGD.")
    if puzzle_path is None :
        raise Exception('puzzle path is empty : must be a valid path .pwp file.')

        
    if planner_path is None :
        print('planner_path is not defined : looking run_planner in the same folder instead...')
        planner_path = next(pathlib.Path(__file__).parent.glob('run_planner*'))
        print('planner path :', planner_path)
        # for key in planner_path :
        #     print(key)
        if planner_path is None :
            raise Exception("run_planner not found in relative folder.")
    output = subprocess.run([fr"{planner_path}", mode, fr"{puzzle_path}"], capture_output=True, text=True).stdout
    updateMessageQueue(f"Solution : {output}", queue)
    return output

   

# def main() :

#     print(type(current_window))

#     user_input = input('level : ')


#     # profiler.enable()
#     converted_level = convertDataToUseable(user_input)
#     [solutions, visited_state_list] = BFS(converted_level)
#     # profiler.disable()
#     for array in solutions :
#         print(array)

#     print(len(visited_state_list))

#     # profiler.print_stats()

#     print(f"elapsed time : {getElapsedTime():.2f} seconds")

#     return 0



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


    modified_data['goalObjects'] = tuple(modified_data['goalObjects']) #knows the goal index
    if not (modified_data['walls'][0]['id'] == 'w') :
        temp_array = [modified_data['walls'][1], modified_data['walls'][0]] #swap
        modified_data['walls'] = temp_array

    #for computing the relative objects moveables in which they collision with each other
    moveables_collision_relative = {}
    for moveable in modified_data['moveables'] :
        for other_moveable in modified_data['moveables'] :
            obj = moveable
            other_obj = other_moveable
            if obj == other_obj : continue
            if not(obj['id'] in moveables_collision_relative) : #if dont contain the ids yet make a key with that id
                moveables_collision_relative[obj['id']] = {}

            if not(other_obj['id'] in moveables_collision_relative) : #if dont contain the ids yet make a key with that id
                moveables_collision_relative[other_obj['id']] = {}

            #if the data is already there
            if other_obj['id'] in moveables_collision_relative[obj['id']] and obj['id'] in moveables_collision_relative[other_obj['id']] : continue
            moveables_collision_relative[obj['id']][other_obj['id']] = set()
            moveables_collision_relative[other_obj['id']][obj['id']] = set()
            
            
            #look for longest pixel
            max_x = next(iter(obj['boundaryPixels']))[0]
            max_y = next(iter(obj['boundaryPixels']))[1]
            for pixel in obj['boundaryPixels'] : #index 0 = x, index 1 = y
                if pixel[0] > max_x :
                    max_x = pixel[0]
                if pixel[1] > max_y :
                    max_y = pixel[1]

            for pixel in other_obj['boundaryPixels'] : #index 0 = x, index 1 = y
                if pixel[0] > max_x :
                    max_x = pixel[0]
                if pixel[1] > max_y :
                    max_y = pixel[1]
        
            grid = [max_x + 1, max_y + 1]

            for i in range(grid[0]) :
                for j in range(grid[1]) : 
                    for k in range(grid[0]) :
                        for l in range(grid[1]) :  
                            relative = (i - k, j - l)
                            if relative in moveables_collision_relative[moveable['id']][other_moveable['id']] : continue
                            for pixel in obj['boundaryPixels'] :
                                if (pixel[0] + relative[0], pixel[1] + relative[1]) in other_obj['boundaryPixels'] :
                                    if not (relative in moveables_collision_relative[moveable['id']][other_moveable['id']]) :
                                        moveables_collision_relative[moveable['id']][other_moveable['id']].add(relative)
                                        moveables_collision_relative[other_moveable['id']][moveable['id']].add((relative[0] * -1, relative[1] * -1))
                                        # print('found redundant at : A =', [i, j], ', B =' ,[k, l], ', relative =',relative)
                                    else :
                                        pass
                                        # print('collision at relative : A =', [i, j], ', B =' ,[k, l], ', relative =',relative)
                                    break

    modified_data['moveables_collision_relative'] = moveables_collision_relative
    for obj_id in moveables_collision_relative :
        print(f"{obj_id} : \n {moveables_collision_relative[obj_id]}")
    return modified_data



def simulateStep(pushworld, state, displacement, absolute_moveables_pixels, id_to_index) : #pushworld = dict
    [pushed_object_ids, transitive_stopping] = getPushedObjects(pushworld, state, displacement, absolute_moveables_pixels, id_to_index) #state tuples
    if not transitive_stopping :
        next_state = []

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

            if obj['id'] == 'a' :
                if after_displacement in absolute_actor_walls : 
                    transitive_stopping = True
                    break

        after_displacement = (state[id_to_index[obj['id']]][0] + displacement[0], state[id_to_index[obj['id']]][1] + displacement[1])
        for key in pushworld['moveables_collision_relative'][obj['id']] :
            if key in pushed_object_ids : continue
            
            relative = (after_displacement[0] - state[id_to_index[key]][0],after_displacement[1] - state[id_to_index[key]][1])
            # print(f"{state[id_to_index[key]]} - {after_displacement} = {relative}")
            if relative in pushworld['moveables_collision_relative'][obj['id']][key] : #a m0
                moving_parts.append(pushworld['moveables'][id_to_index[key]])
            #     print(key + ' - ' + obj['id'] + ' : collision')
            # else :
            #     print(key + ' - ' + obj['id'] +' : no collision')

            
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

