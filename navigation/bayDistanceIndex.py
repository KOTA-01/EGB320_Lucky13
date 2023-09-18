def distance_from_wall(bay_number):
	bay_distance = {
		0: 0.91,
		1: 0.65,
		2: 0.39,
		3: 0.13
	}
	return bay_distance.get(bay_number, 0)