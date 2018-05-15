# Sliding 15-Puzzle
import sys
import random
from Queue import PriorityQueue
import time

#-------------------------------------Walking Distance starts here--------------------------------------
# Converts 15-puzzle to WD matrix
# Vertical Matrix Conversion
def wdV(state):
    wdVlist = []
    for j in range(4):
        one = 0
        two = 0
        three = 0
        four = 0
        for i in range(4):
            if state[i+(4*j)] >= 1 and state[i+(4*j)] <= 4:
                one += 1
            elif state[i+(4*j)] <= 8:
                two += 1
            elif state[i+(4*j)] <=12:
                three += 1
            elif state[i+(4*j)] <= 15:
                four += 1
        wdVlist.extend((one, two, three, four))
    return wdVlist

# Horizontal Matrix Conversion
def wdH(state):
    wdHlist = []
    for j in range(4):
        one = 0
        two = 0
        three = 0
        four = 0
        for i in range(4):
            if state[j+(4*i)] % 4 == 1:
                one += 1
            elif state[j+(4*i)] % 4 == 2:
                two += 1
            elif state[j+(4*i)] % 4 == 3:
                three += 1
            elif state[j+(4*i)] != 16:
                four += 1
        wdHlist.extend((one, two, three, four))
    return wdHlist


#Neighbor function
def WDneighbor(WDstate):
    neighborhood = []
    newState = []

    #find row with blank
    blankAt = -1
    while blankAt == -1:
        for i in range(4):
            total = 0
            for j in range(4):
                total += WDstate[(4*i)+j]
            if total == 3:
                blankAt = i

    # swapping the blank up
    if blankAt != 0:
        for i in range(4):
            if WDstate[(blankAt-1)*4 + i] != 0:
                #minus 1 in row above that doesn't equal to 0, and add 1 to corresponding position in the blank's row
                newState = WDstate[:((blankAt-1)*4 + i)] + [WDstate[(blankAt-1)*4 + i]-1] + WDstate[((blankAt-1)*4 + i+1):]
                newState[(blankAt*4)+i] += 1
                neighborhood.append(newState)

    # swapping the blank down
    if blankAt != 3:
        for i in range(4):
            if WDstate[(blankAt+1)*4 + i] != 0:
                #minus 1 in row below that doesn't equal to 0, and add 1 to corresponding position in the blank's row
                newState = WDstate[:((blankAt+1)*4 + i)] + [WDstate[(blankAt+1)*4 + i]-1] + WDstate[((blankAt+1)*4 + i+1):]
                newState[(blankAt*4) +i] += 1
                neighborhood.append(newState)
    return neighborhood


#breadth first search with the root being the goal state
#allWD is the final dictionary that will be used in the heuristic
def BFS(goalState, neighborFn, allWD):
    frontier = []
    frontier.append([goalState, 0])
    allWD.update({str(goalState):0})
    while len(frontier) > 0:
        node = frontier.pop(0)
        neighborhood = neighborFn(node[0])
        for neighbor in neighborhood:
            if str(neighbor) not in allWD:
                frontier.append([neighbor, node[1]+1])
                allWD.update({str(neighbor):(node[1]+1)})
    return allWD

#-----------------------------Walking Distance ends here---------------------------------------

def isGoal(state):
    return state == [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

def neighbors(state):
    neighborhood = []

    # find blank position
    i = state.index(16)

    # move blank left?
    if i % 4 != 0:
        newState = state[:i-1] + [state[i],state[i-1]] + state[i+1:]
        neighborhood.append(newState)

    # move blank right?
    if i % 4 != 3:
        newState = state[:i] + [state[i+1],state[i]] + state[i+2:]
        neighborhood.append(newState)

    # move blank up?
    if i > 3:
        newState = state[:i-4] + [state[i]] + state[i-3:i] + [state[i-4]] + state[i+1:]
        neighborhood.append(newState)

    # move blank down?
    if i < 12:
        newState = state[:i] + [state[i+4]] + state[i+1:i+4] + [state[i]] + state[i+5:]
        neighborhood.append(newState)

    return neighborhood


def print15(state):
    for row in range(4):
        for col in range(4):
            if state[4*row+col] < 10:
                    sys.stdout.write(" ")
            sys.stdout.write(str(state[4*row+col]))
            sys.stdout.write("\t")
        print("")


def print15s(path):
    for i, state in enumerate(path):
        print("step " + str(i))
        print15(state)
        print("")


#modified so doesn't generate previous states
def scrambler(state, n):
    step = 0
    visited = []
    while step < n:
        neighborList = neighbors(state)
        num = len(neighborList)
        nextNeighbor = neighborList[random.randint(0, num-1)]
        if rankPerm(nextNeighbor) in visited: continue
        else:
            visited.append(rankPerm(nextNeighbor))
            state = nextNeighbor
            step += 1
    return state


#heuristic for Manhattan Distance forward search in bidirectional
def heuristicMDForward(state):
    total = 0
    for row in range(4):
        for col in range(4):
            index = 4*row + col
            correct = index+1
            incorrect = state[index]
            if incorrect == 16: continue
            incorrectRow = int((incorrect-1)/4)
            incorrectCol = (incorrect-1)%4
            distance = abs(incorrectRow-row) + abs(incorrectCol-col)
            total += distance
    return total


#heuristic for Manhattan Distance backward search in bidirectional
def heuristicMDBack(currentState, startState):
    total = 0
    for i in range(16):
        ci = currentState.index(i+1)
        si = startState.index(i+1)
        if ci != si:
            total += abs(si%4 - ci%4) + abs(si/4 - ci/4)
    return total-1


def heuristicWD(state):
    total = []
    #convert the state to walking distance matrices
    wdVlist = wdV(state)
    wdHlist = wdH(state)
    #search for the vertical and horizontal cost in the dictionary and then add together
    #this is for the forward search
    ftotal = WDdic[str(wdVlist)] + WDdic[str(wdHlist)]
    #this is for the backward search
    btotal = WDcurrentV[str(wdVlist)] + WDcurrentH[str(wdHlist)]
    #return a list of two elements
    total.append(ftotal)
    total.append(btotal)
    return total


def AStar(S, goal, neighborhoodFn, visitFn, hWD, hMDF, hMDB):
    global maxTime
    startTime = time.time()

    # two searches, frontier is the forward search, and backward is the backward search
    frontier = PriorityQueue()
    backward = PriorityQueue()

    # dictionaries of explored states for each direction
    # keys are the ranks of each state, and values are the length of the path to the start/goal state
    fexplored = {str(S[0]): 0}
    bexplored = {str(goal[0]): 0}

    # rank of the goal and start state, to be used later for comparison
    goalRank = str(goal[0])
    startRank = str(S[0])

    # add goal and start states to the two queues
    for s in S:
        frontier.put((0, [s]))
    for g in goal:
        backward.put((0, [g]))

    while frontier.qsize() > 0 and backward.qsize() > 0:

        # check time
        currentTime = time.time()
        if currentTime - startTime > maxTime:
            return [-1, None]

        #forward  search
        (_, fpath) = frontier.get()
        fnode = fpath[-1]
        frank = str(fnode)

        # if current node is goal state, return success
        if frank  == goalRank:
            currentTime = time.time()
            return [currentTime - startTime, len(fpath)]

        # if current node already been explored in the other directions, return total path length
        elif frank in bexplored:
            currentTime = time.time()
            return [currentTime - startTime, len(fpath)+bexplored[frank]-1]

        # branch and bound - skip path that has length more than 40
        elif len(fpath) > 41: continue

        # else explore the current node
        else:
            neighborhood = neighborhoodFn(fnode)
            for neighbor in neighborhood:
                if str(neighbor) not in fexplored:
                    newPath = fpath + [neighbor]
                    pastCost = len(newPath)-1
                    #past cost is the value of the dictionary key, so the path length of this state can be determined
                    fexplored.update({str(neighbor): pastCost})
                    #calculate future cost using Manhattan Distance and Walking Distance, use the higher cost
                    WDfc = hWD(neighbor)[0]
                    MDfc = hMDF(neighbor)
                    if WDfc > MDfc:
                        futureCost = WDfc
                    else:
                        futureCost = MDfc
                    totalCost = pastCost + futureCost
                    frontier.put((totalCost, newPath))

        #backward search
        (_, bpath) = backward.get()
        bnode = bpath[-1]
        brank = str(bnode)

        # if current node is start state, return success
        if brank == startRank:
            currentTime = time.time()
            return [currentTime - startTime, len(bpath)]

        # if current node already been explored in the other directions, return total path length
        elif brank in fexplored:
            currentTime = time.time()
            return [currentTime - startTime, len(bpath)+fexplored[brank]-1]

        # branch and bound - skip path that has length more than 40
        elif len(bpath) > 41: continue

        # else explore the current node
        else:
            neighborhood = neighborhoodFn(bnode)
            for neighbor in neighborhood:
                if str(neighbor) not in bexplored:
                    newPath = bpath + [neighbor]
                    pastCost = len(newPath)-1
                    #past cost is the value of the dictionary key, so the path length of this state can be determined
                    bexplored.update({str(neighbor): pastCost})
                    #calculate future cost using Manhattan Distance and Walking Distance, use the higher cost
                    WDfc = hWD(neighbor)[1]
                    MDfc = hMDB(neighbor, S[0])
                    if WDfc > MDfc:
                        futureCost = WDfc
                    else:
                        futureCost = MDfc
                    totalCost = pastCost + futureCost
                    backward.put((totalCost, newPath))

    return [-1, None]

# rankPerm(perm) returns the rank of permutation perm.
# The rank is done according to Myrvold, Ruskey "Ranking and unranking permutations in linear-time".
# perm should be a 1-based list, such as [1,2,3,4,5].
def rankPerm(perm, inverse = None, m = None):
    # if the parameters are None, then this is the initial call, so set the values
    if inverse == None:
        perm = list(perm) # make a copy of the perm; this algorithm will sort it
        m = len(perm)
        inverse = [-1]*m
        for i in range(m):
            inverse[perm[i]-1] = i+1

    if m == 1:
        return 0
    s = perm[m-1]-1
    x = m-1
    y = inverse[m-1]-1
    temp = perm[x]
    perm[x] = perm[y];
    perm[y] = temp;
    x = s
    y = m-1
    temp = inverse[x]
    inverse[x] = inverse[y]
    inverse[y] = temp
    return s + m * rankPerm(perm, inverse, m-1)


def isSolvable(state):
    # find blank position
    z = state.index(16)

    invs = 0
    for i in range(15):
        if i == z: continue
        for j in range(i+1,16):
            if j == z: continue
            if state[i] > state[j]:
                invs += 1

    return (z/4 + invs) % 2 == 1


def doNothing(path):
    pass


if __name__ == "__main__":
    #generate the Walking Distance dictionary first
    WDdic = {}
    WDgoalState = [4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 3]
    BFS(WDgoalState, WDneighbor, WDdic)

    global maxTime

    solved5 = 0
    solved30 = 0
    solved100 = 0

    maxTime = 100
    numTests = 100

    for test in range(numTests):

        # Make a random state.
        state = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
        goalState = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]


        random.shuffle(state)
        while not isSolvable(state):
            random.shuffle(state)

        # generate the two dictionaries for the walking distance heuristic
        WDcurrentV = {}
        WDcurrentH = {}
        Vstate = wdV(state)
        Hstate = wdH(state)
        BFS(Vstate, WDneighbor, WDcurrentV)
        BFS(Hstate, WDneighbor, WDcurrentH)

        print("\nRunning test " + str(test+1) + " out of " + str(numTests))
        print15(state)
        print(str(state))
        print("has rank " + str(rankPerm(state)))
        [runTime, path] = AStar([state], [goalState], neighbors, doNothing, heuristicWD, heuristicMDForward, heuristicMDBack)
        if runTime == -1:
            print("no solution found")
            continue
        else:
            #print("solved in " + str(len(path)-1) + " moves")
            print("solved in " + str(path) + " moves")
            print("solved in " + str(runTime) + " seconds")

        if runTime <= 101:
            solved100 += 1
        if runTime <= 30:
            solved30 += 1
        if runTime <= 5:
            solved5 += 1

    print("Solved in 5 seconds: " + str(solved5) + "/" + str(numTests))
    print("Solved in 30 seconds: " + str(solved30) + "/" + str(numTests))
    print("Solved in 100 seconds: " + str(solved100) + "/" + str(numTests))
