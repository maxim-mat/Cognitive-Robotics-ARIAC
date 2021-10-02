import search
import random
import math


class AriacProblem(search.Problem):

    def __init__(self, initial_input):
        """
        search.Problem.__init__(self, initial) creates the root node"""

        # set the state
        (kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit,avilable_parts) = initial_input
        initial_state = initial_input
        #print(initial_input)
        search.Problem.__init__(self, initial_state)


    def actions(self, state):
        """Return the actions that can be executed in the given
        state. """
        kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit, avilable_parts = state
        kit_rob_actions = []
        ass_rob_actions = []
        allActions = []
        for p in range(0, len(avilable_parts)):
            for s in range(0, len(parts_required_kit)):
                if avilable_parts[p][2] in parts_required_kit[s]:
                    kit_rob_actions.append(("kit_rob", "kit", s, p))
                    ass_rob_actions.append(("ass_rob", "kit", s, p))
            for s in range(0, len(parts_required_ass)):
                if avilable_parts[p][2] in parts_required_ass[s]:
                    ass_rob_actions.append(("ass_rob", "ass", s, p))
        allActions = kit_rob_actions + ass_rob_actions
        for k in kit_rob_actions:
            for a in ass_rob_actions:
                if (a[1] == "ass") and (k[3] != a[3]):
                    allActions.append((k,a))
                elif (a[1] == "kit") and (k[3] != a[3]):
                    allActions.append((k, a))

        return tuple(allActions)


    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. """
        kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit, avilable_parts = state
        new_kit_rob_loc = list(kit_rob_loc)
        new_ass_rob_loc = list(ass_rob_loc)
        new_ass_stations_loc = list(ass_stations_loc)
        new_kit_stations_loc = list(kit_stations_loc)
        new_parts_required_ass = list(parts_required_ass)
        new_parts_required_kit = list(parts_required_kit)
        new_avilable_parts = list(avilable_parts)

        if len(action) == 2:
            actions = []
            if action[0][3] > action[1][3]:
                actions = [action[0], action[1]]
            else:
                actions = [action[1], action[0]]
            for act in actions:
                # update rob loc
                if act[0] == "ass_rob":
                    if act[1] == "kit":
                        new_ass_rob_loc = new_kit_stations_loc[act[2]]
                    elif act[1] == "ass":
                        new_ass_rob_loc = new_ass_stations_loc[act[2]]
                elif act[0] == "kit_rob":
                    if act[1] == "kit":
                        new_kit_rob_loc = new_kit_stations_loc[act[2]]
                    elif act[1] == "ass":
                        new_kit_rob_loc = new_ass_stations_loc[act[2]]

                # update parts required
                part = new_avilable_parts[act[3]]
                if act[1] == "kit":
                    temp = list(new_parts_required_kit[act[2]])
                    for t in range(0, len(temp)):
                        if temp[t] == part[2]:
                            del temp[t]
                            break
                    new_parts_required_kit[act[2]] = tuple(temp)
                elif act[1] == "ass":
                    temp = list(new_parts_required_ass[act[2]])
                    for t in range(0, len(temp)):
                        if temp[t] == part[2]:
                            del temp[t]
                            break
                    new_parts_required_ass[act[2]] = tuple(temp)

                # update avilable parts
                del new_avilable_parts[act[3]]
        else:
            # update rob loc
            if action[0] == "ass_rob":
                if action[1] == "kit":
                    new_ass_rob_loc = new_kit_stations_loc[action[2]]
                elif action[1] == "ass":
                    new_ass_rob_loc = new_ass_stations_loc[action[2]]
            elif action[0] == "kit_rob":
                if action[1] == "kit":
                    new_kit_rob_loc = new_kit_stations_loc[action[2]]
                elif action[1] == "ass":
                    new_kit_rob_loc = new_ass_stations_loc[action[2]]

            # update parts required
            part = new_avilable_parts[action[3]]
            if action[1] == "kit":
                temp = list(new_parts_required_kit[action[2]])
                for t in range(0,len(temp)):
                    if temp[t] == part[2]:
                        del temp[t]
                        break
                new_parts_required_kit[action[2]] = tuple(temp)
            elif action[1] == "ass":
                temp = list(new_parts_required_ass[action[2]])
                for t in range(0, len(temp)):
                    if temp[t] == part[2]:
                        del temp[t]
                        break
                new_parts_required_ass[action[2]] = tuple(temp)

            # update avilable parts
            del new_avilable_parts[action[3]]



        newState = (tuple(new_kit_rob_loc),tuple(new_ass_rob_loc),tuple(new_ass_stations_loc),tuple(new_kit_stations_loc),tuple(new_parts_required_ass),tuple(new_parts_required_kit),tuple(new_avilable_parts))
        return newState


    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        out = False
        kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit,avilable_parts = state
        required_parts = []
        for s in parts_required_kit:
            for p in s:
                required_parts.append(p)
        for s in parts_required_ass:
            for p in s:
                required_parts.append(p)

        if len(required_parts) == 0:
            out = True
            return out
        if len(avilable_parts) == 0:
            return True
        out = True
        for p in avilable_parts:
            if p[2] in required_parts:
                return False
        return out


    def h(self, node):
        """ This is the heuristic. It gets a node and returns a goal distance estimate
        The estimate is the amount of remaining required parts (how many more parts can we ship)"""
        kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit, avilable_parts = node.state
        required_parts = []
        for s in parts_required_kit:
            for p in s:
                required_parts.append(p)
        for s in parts_required_ass:
            for p in s:
                required_parts.append(p)
        avilable_required_parts = []
        for p in avilable_parts:
            if p[2] in required_parts:
                avilable_required_parts.append(p[2])
        h_count = 0
        for s in required_parts:
            for i in range(0, len(avilable_required_parts)):
                if s == avilable_required_parts[i]:
                    h_count = h_count + 1
                    del avilable_required_parts[i]
                    break
        return h_count


    def path_cost(self, c, state1, action, state2):
        kit_rob_loc, ass_rob_loc, ass_stations_loc, kit_stations_loc, parts_required_ass, parts_required_kit, avilable_parts = state1

        cost_t = 0
        if len(action) == 2:
            for act in action:
                loc1 = None
                loc2 = None
                loc3 = None
                if act[0] == "ass_rob":
                    loc1 = ass_rob_loc
                elif act[0] == "kit_rob":
                    loc1 = kit_rob_loc
                loc2 = avilable_parts[act[3]]
                if act[1] == "ass":
                    loc3 = ass_stations_loc[act[2]]
                elif act[1] == "kit":
                    loc3 = kit_stations_loc[act[2]]
                cost_t = cost_t + two_point_distance(loc1,loc2) + two_point_distance(loc2,loc3)
            cost_t = cost_t / 2
        else:
            loc1 = None
            loc2 = None
            loc3 = None
            if action[0] == "ass_rob":
                loc1 = ass_rob_loc
            elif action[0] == "kit_rob":
                loc1 = kit_rob_loc
            loc2 = avilable_parts[action[3]]
            if action[1] == "ass":
                loc3 = ass_stations_loc[action[2]]
            elif action[1] == "kit":
                loc3 = kit_stations_loc[action[2]]
            cost_t = cost_t + two_point_distance(loc1, loc2) + two_point_distance(loc2, loc3)
        return cost_t


def two_point_distance(point1,point2):
    distance = 0
    distance = math.sqrt(((point1[0]-point2[0])**2) + ((point1[1]-point2[1])**2))
    return distance


def create_ariac_problem(game):
    return AriacProblem(game)

