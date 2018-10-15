#! /usr/bin/env python
# Quick-and-dirty odometry change checking script

import math

def odochange(theta, left, right):
	x = math.cos(theta) * 0.1 * 0.0278 * (left + right) / 2
	y = math.sin(theta) * 0.1 * 0.0278 * (left + right) / 2
	dtheta = (right - left) * 0.1 * 0.0278 / 0.0865
	return (x, y, dtheta)

directions = {"east": 0, "north": math.pi/2, "south": -math.pi/2, "west": math.pi}
moves = {"forward": (1, 1), "backward": (-1, -1)}
turns = {"left": (-1, 1), "right": (1, -1)}

ways = {f"Moving {move} {direction}":odochange(directions[direction], moves[move][0], moves[move][1]) for move in moves for direction in directions} 
otherways = {f"Turning {turn}": odochange(0, turns[turn][0], turns[turn][1]) for turn in turns}

ways.update(otherways)
for k in ways:
	print(k, ways[k])