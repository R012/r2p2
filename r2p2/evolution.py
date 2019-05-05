#!/usr/bin/env python


import random
import time
import os
import inspyred

history = open('../logs/history_'+str(int(time.time()))+'.log', 'a+')
cur_best = 0

def generate_const(random, args):
	min_gen = args.get('min_gen', 0)
	max_gen = args.get('max_gen', 2)
	size = args.get('size', 10)
	return [0.1] * size

def generate_float(random, args):
	min_gen = args.get('min_gen', 0)
	max_gen = args.get('max_gen', 2)
	size = args.get('size', 10)
	return [random.uniform(min_gen,max_gen) for i in range(size)]

def write_params(string):
	f = open('../res/weights.json', 'w+')
	f.write('{"params": '+string+'}')
	f.close()

@inspyred.ec.evaluators.evaluator
def evaluate_ann(candidate, args):
	global history, cur_best
	string = ''.join(str(candidate)).replace(" ", "")
	#print("Evaluating: " + string)
	try:
		write_params(string)
		original_time = os.path.getmtime('../res/fitness.txt')
		while os.path.getmtime('../res/fitness.txt') == original_time:
			write_params(string)
			time.sleep(0.5)
		f = open('../res/fitness.txt', 'r')
		output = f.read()
		f.close()
		output = float(output)
		print("Fitness: " + str(output))
		if output >= cur_best:
			history.write('\n\n['+str(output)+']: '+string)
			cur_best = output
		return output
	except Exception as e:
		print(__file__)
		print(e)
		return(0)

@inspyred.ec.evaluators.evaluator
def evaluate_test(candidate, args):
		return sum(candidate)

if __name__ == "__main__":
    # This main fuction is provided as a trivial functional example
    # TODO: Implement your EA here
    rand = random.Random()
    rand.seed(int(time.time()))
    
    length = 7
    weights = [random.uniform(-1, 1) for i in range(length)]
    print("Assessing network")
    print(weights)
    evaluate_ann([weights], None)
