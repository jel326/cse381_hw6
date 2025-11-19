from constants import *
from utils import *
from core import *

import pdb
import copy
from functools import reduce

from statesactions import *

############################
## HELPERS

### Return true if the given state object is a goal. Goal is a State object too.
def is_goal(state, goal):
	return len(goal.propositions.difference(state.propositions)) == 0

### Return true if the given state is in a set of states.
def state_in_set(state, set_of_states):
	for s in set_of_states:
		if s.propositions != state.propositions:
			return False
	return True

### For debugging, print each state in a list of states
def print_states(states):
	for s in states:
		ca = None
		if s.causing_action is not None:
			ca = s.causing_action.name
		print(s.id, s.propositions, ca, s.get_g(), s.get_h(), s.get_f())


############################
### Planner 
###
### The planner knows how to generate a plan using a-star and heuristic search planning.
### It also knows how to execute plans in a continuous, time environment.

class Planner():

	def __init__(self):
		self.running = False              # is the planner running?
		self.world = None                 # pointer back to the world
		self.the_plan = []                # the plan (when generated)
		self.initial_state = None         # Initial state (State object)
		self.goal_state = None            # Goal state (State object)
		self.actions = []                 # list of actions (Action objects)

	### Start running
	def start(self):
		self.running = True
		
	### Stop running
	def stop(self):
		self.running = False

	### Called every tick. Executes the plan if there is one
	def update(self, delta = 0):
		result = False # default return value
		if self.running and len(self.the_plan) > 0:
			# I have a plan, so execute the first action in the plan
			self.the_plan[0].agent = self
			result = self.the_plan[0].execute(delta)
			if result == False:
				# action failed
				print("AGENT FAILED")
				self.the_plan = []
			elif result == True:
				# action succeeded
				done_action = self.the_plan.pop(0)
				print("ACTION", done_action.name, "SUCCEEDED")
				done_action.reset()
		# If the result is None, the action is still executing
		return result

	### Call back from Action class. Pass through to world
	def check_preconditions(self, preconds):
		if self.world is not None:
			return self.world.check_preconditions(preconds)
		return False

	### Call back from Action class. Pass through to world
	def get_x_y_for_label(self, label):
		if self.world is not None:
			return self.world.get_x_y_for_label(label)
		return None

	### Call back from Action class. Pass through to world
	def trigger(self, action):
		if self.world is not None:
			return self.world.trigger(action)
		return False

	### Compute the heuristic value of the current state using the HSP technique.
	### Current_state and goal_state are State objects.
	def compute_heuristic(self, current_state, goal_state, actions):
		actions = copy.deepcopy(actions)  # Make a deep copy just in case
		h = 0                             # heuristic value to return
		### YOUR CODE BELOW
		#1. construct teh relaxed graph
		dummy_start = Action(
			"dummy_start",
			preconditions=set(),
			add_list=set(current_state.propositions),		#true in our current state
			delete_list=set(),
			cost=0
		)

		dummy_goal = Action(		#dummy goal
			"dummy_goal",
			preconditions=set(goal_state.propositions),		#goal propositions
			add_list=set(),
			delete_list=set(),
			cost=0
		)
  
		all_action = [dummy_start] + actions + [dummy_goal]		#all action nodes in the graph
  
		incoming_edges = {}		#list where a1 lists has prop
		outgoing_edges = {}		#list where a2 lists preconditons have prop
		for a in all_action:
			incoming_edges[a] = []
			outgoing_edges[a] = []
		for a1 in all_action:
			for a2 in all_action:
				if a1 is not a2:
					for prop in a1.add_list:
						if prop in a2.preconditions:
							incoming_edges[a2].append((a1, prop))
							outgoing_edges[a1].append((a2, prop))
		#2. walk teh graph and find the longest path between dummy start and dummy goal
		queue = [dummy_start]		#queue for actions
		visited = set([dummy_start])		#visited set of actions
		dist = {dummy_start: 0}			#distance from dummy_start to that action
		edge_value = {}			#values for edges 
		max_edge_value = 0			#0 as a placeholder for now
  
		for (child, prop) in outgoing_edges[dummy_start]:
			#outgoing edges from dummy sart
			e_val = dist[dummy_start] + dummy_start.cost + 1
			edge_value[(dummy_start, child, prop)] = e_val
			if e_val > max_edge_value:		#swap our max edge value to be consistentn
				max_edge_value
  
		while len(queue) > 0:
			current_action = queue.pop(0)
   
			for a in all_action:
				if a in visited:
					continue
   
				all_satisfied = True			#checkin if all preconditons are satisfied
				for prop in a.preconditions:
					prop_satisfied = False
					for (pred, edge_prop) in incoming_edges[a]:
						if edge_prop == prop and pred in visited:
							prop_satisfied = True			#comes from at least one incoming edge with propositon coming from a visted action
							break
					if not prop_satisfied:
						all_satisfied = False
						break
    
				if all_satisfied:		#all predecessors for required proposition are visited 
					max_incoming = 0
					for (pred, edge_prop) in incoming_edges[a]:
						if pred in visited:
							key = (pred,a, edge_prop)
							if key in edge_value:
								if edge_value[key] > max_incoming:
									max_incoming = edge_value[key]
					dist[a] = max_incoming
					visited.add(a)		#mark the action as visited and then push it
					queue.append(a)
     
					for (child, prop) in outgoing_edges[a]:		#outgoing edges from teh action a
						e_val = dist[a] + a.cost + 1
						edge_value[(a, child, prop)] = e_val
						if e_val > max_edge_value:
							max_edge_value = e_val
		h = max_edge_value		#after the walk, the h value should be the biggest edge cost from the walk after the queue has been empty
		### YOUR CODE ABOVE
		return h

	### Generate a plan. Init and goal are State objects. Actions is a list of Action objects
	### Return the plan and the closed list
	def astar(self, init, goal, actions):
		plan = []	# the final plan
		open = [init]	# the open list (priority queue) holding State objects
		closed = []  	# the closed list (already visited states). Holds state objects
		closed = set()
		current = init

		def successors(state, actions, planner):
			sucs = []
			for act in actions:
				if len(act.preconditions.difference(state.propositions)) == 0:
					# action is applicable
					new_state = State(state.propositions.difference(act.delete_list).union(act.add_list))
					new_state.parent = state
					new_state.causing_action = act
					new_state.g = state.g + act.cost
					new_state.h = planner.compute_heuristic(state, planner.goal_state, planner.actions)
					sucs.append(new_state)
			return sucs
 
		while current is not None and not is_goal(current, goal) and len(open) > 0:
				if current.causing_action is not None:
						print("current", current.id, current.causing_action.name, current.propositions, current.get_g(), current.get_h(), current.get_f())
				else:
					print("current", current.id, None, current.propositions, current.get_g(), current.get_h(), current.get_f())

				closed.add(current)
				sucs = successors(current, actions, self)
				open.pop(0)
				for s in sucs:
						if not state_in_set(s, closed):
							open.append(s)
				open = sorted(open, key = lambda s:s.get_f())	#sorting open by f
  
				if len(open) > 0:		#pick the next state
					current = open[0]
				else:
						current = None
		if current is not None and is_goal(current, goal): # success
			while current.parent is not None:
				plan.append(current.causing_action)
				current = current.parent
			plan = list(reversed(plan))
		closed = list(closed)
		return plan, closed
