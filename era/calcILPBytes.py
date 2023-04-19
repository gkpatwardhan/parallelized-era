f = open("era1_output.txt");
current_node = "HOST"
byte_transfers = {}
last_modifier = {}
parser_state = -1
for line in f:
	ty = len(line.split(":")[1])
	if ty == 1:
		parser_state = (parser_state + 1) % 3
		if parser_state == 0:
			current_node = line.split(":")[0]
			byte_transfers[current_node] = {}
	else:
		assert(parser_state != 0);
		name = line.split(":")[0]
		amount = int(line.split(" ")[1])
		if parser_state == 1:
			if name in last_modifier:
				if not last_modifier[name] in byte_transfers[current_node]:
					byte_transfers[current_node][last_modifier[name]] = 0
				byte_transfers[current_node][last_modifier[name]] += amount
			else:
				if not "HOST" in byte_transfers[current_node]:
					byte_transfers[current_node]["HOST"] = 0
				byte_transfers[current_node]["HOST"] += amount
		else:
			last_modifier[name] = current_node

for k, v in byte_transfers.items():
	print(k, end='')
	print(" ___", end='')
	for k2, v2 in v.items():
		if k2 == "HOST":
			continue
		print(" ", end='')
		print(k2, end='')
		print(" ", end='')
		print(v2, end='')
	print(" HOST ", end='')
	if not "HOST" in v:
		v["HOST"] = 0
	print(v["HOST"], end='')
	print("")
