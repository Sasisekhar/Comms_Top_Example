#pip install diagrams
#sudo apt install graphviz


import os, fnmatch, sys, json, re, copy
from colorama import Fore
from itertools import chain

import pdb

def find(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result

def getHeaders(path):
	header_path = []
	path_to_file = ""
	for i in path.split('/')[:-1]:
		path_to_file += i + '/'

	try:
		fp = open(path)
	except PermissionError:
		print(Fore.RED + "Not enough permissions")
		sys.exit()
	except Exception as e:
		# print(Fore.RED + "Error reading file:", i, "-", e)
		pass
	else:
		with fp:
			headers = []
			for i in fp.readlines():
				try:
					if(i.lstrip()[0] == '#'):
						headers.append(i)
				except:
					pass
			for i in headers:
				if "<" in i:
					header_path.append(path_to_file + (i.split("<")[1]).split(">")[0])

				elif "\"" in i:
					header_path.append(path_to_file + (i.split("\"")[1]).split("\"")[0])
		fp.close()

	return header_path

def getUserFiles(path):
	returnList = []
	key = ""
	for i in path:
		if('cadmium' in i.split('/')):
			if('logger' in i.split('/')):
				key = "Main"
			elif('coupled.hpp' in i.split('/') and key != "Main"):
				key = "Coupled"
			elif('atomic.hpp' in i.split('/') and key != "Main"):
				key = "Atomic"

			print(Fore.YELLOW + "Skipping \"", i, "\"(Internal library file)")

		else:
			try:
				fp = open(i)
			except PermissionError:
				# print(Fore.RED + "Not enough permissions")
				sys.exit()
			except Exception as e:
				# print(Fore.RED + "Error reading file:", i, "-", e)
				pass
			else:
				returnList.append(i)
				fp.close()

	return returnList, key

def getAllFiles(files):
	return_list = []

	for file in files:
		header_paths = getHeaders(file)
		user_files, key = getUserFiles(header_paths)

		if(user_files != []):
			return_list.extend([getAllFiles(user_files), {key : file}])
		else:
			return_list.append({key : file})

	return return_list

def printModels(arr, indent = 0):
	for item in arr:
		if isinstance(item, list):
			printModels(item, indent + 1)
		else:
			print('\t' * indent, item)

model_stack = []

def convertToDictionary(arr, depth = 0):
	main_stack = []
	global model_stack

	for item in arr:
		if isinstance(item, list):		
			convertToDictionary(item, depth + 1)
		else:
			if("Main" in item.keys()):
				main_stack.append({"Main" : item["Main"], "depth" : depth, "Top" : copy.deepcopy(model_stack)})
				model_stack.clear()
			elif("Coupled" in item.keys()):
				tmp = []
				for atomic in model_stack:
					if atomic['depth'] > depth:
						tmp.append(atomic)
				
				model_stack = [x for x in model_stack if x not in tmp]

				IC  = {}
				try:
					fp = open(item["Coupled"])
				except PermissionError:
					print(Fore.RED + "Not enough permissions")
					sys.exit()
				except Exception as e:
					print(Fore.RED + "Error reading file:", atomic, "-", e)
				else:
					with fp:
						for i in fp.readlines():
							if(i.lstrip().split('(')[0] == 'addCoupling'):
								from_model = i.lstrip().split('(')[1].split(')')[0].strip().split(',')[0].strip()
								to_model = i.lstrip().split('(')[1].split(')')[0].strip().split(',')[-1].strip()
								IC.update({from_model : to_model})
					fp.close()

				instance_names = {}
				try:
					fp = open(item["Coupled"])
				except PermissionError:
					print(Fore.RED + "Not enough permissions")
					sys.exit()
				except Exception as e:
					print(Fore.RED + "Error reading file:", atomic, "-", e)
				else:
					with fp:
						for i in fp.readlines():
							if(i.lstrip().split('=')[-1].split('<')[0].strip() == 'addComponent'):
								# atomic_name = i.lstrip().split('=')[-1].split('<', 1)[-1].rsplit('>', 1)[0].strip()
								atomic_name = i.lstrip().split('=')[-1].split('<')[1].split('>')[0].strip()
								instance_name = i.lstrip().split('=')[0].strip().split(" ")[-1]
								instance_names.update({atomic_name : instance_name})
					fp.close()

				EIC = []
				EOC = []
				try:
					fp = open(item["Coupled"])
				except PermissionError:
					print(Fore.RED + "Not enough permissions")
					sys.exit()
				except Exception as e:
					print(Fore.RED + "Error reading file:", atomic, "-", e)
				else:
					with fp:
						for i in fp.readlines():
							if("addInPort<" in i.lstrip()):
								EIC.append(i.lstrip().split('=')[0].strip())
							if("addOutPort<" in i.lstrip()):
								EOC.append(i.lstrip().split('=')[0].strip())
					fp.close()


				model_stack.append({"Coupled" : item["Coupled"], "depth" : depth, "Atomics" : tmp, "Instance_names" : instance_names, "IC" : IC, "EIC" : EIC, "EOC": EOC})
			elif("Atomic" in item.keys()):
				model_stack.append({"Atomic" : item["Atomic"], "depth" : depth})
	
	return main_stack

def getStates(d, target_key):
	result = copy.deepcopy(d)
	for key, value in d.items():
		if key == target_key:
			states = parseStates(value)
			result.update({"states" : states})
		elif (isinstance(value, dict) and not(key == 'IC' or key == 'Instance_names')):
			result.update(getStates(value, target_key))
		elif (isinstance(value, list) and not(key == "EIC" or key == "EOC")):
			listDicts = []
			for item in value:
				listDicts.append(getStates(item, target_key))
			result.update({key : listDicts})
	return result


def parseStates(atomic):
	flag = False
	state_name = ""
	state_vars = {}
	try:
		fp = open(atomic)
	except PermissionError:
		print(Fore.RED + "Not enough permissions")
		sys.exit()
	except Exception as e:
		print(Fore.RED + "Error reading file:", atomic, "-", e)
	else:
		with fp:
			for i in fp.readlines():
				try:
					if re.search(r'struct\s+(\w+)\s*{', i.lstrip()):
						state_name = re.search(r'struct\s+(\w+)\s*{', i.lstrip()).group(1)
						flag = True
					if(flag):
						pattern = re.compile(fr"{re.escape(state_name)}\s*\(.*?\)\s*:\s*((?:\w+\s*\(.*?\),\s*)*\w+\s*\(.*?\))\s*{{")
						if pattern.search(i.lstrip()):
							var_and_val_group = pattern.search(i.lstrip()).group(1)
							data = list(chain.from_iterable(re.findall(r'(\w+)\s*\(([^)]*)\)', var_and_val_group)))
							for i in range(0, len(data) - 1, 2):
								state_vars.update({data[i] : (data[i + 1].strip() if data[i + 1] else "NULL")})
							flag = False
					
				except:
					pass
		fp.close()
	return state_vars




if __name__ == '__main__':
	if 'main' not in os.listdir():
		print(Fore.RED + "main directory not found")
		sys.exit()

	cwd = os.getcwd()

	print("Finding entry point file...")

	main_wd = cwd + '/main'
	main_cpp = find('*main*.cpp', main_wd)

	if (main_cpp == []):
		print(Fore.RED, "Entry point cpp file not found")
		sys.exit()

	print("Finding the models...")

	models = getAllFiles([main_cpp[-1]])
	print(Fore.RESET)

	print("Parsed file system structure:- ")
	printModels(models)

	print("Converting to Dictionary...")
	main_stack = convertToDictionary(models) #Also extracts IC
	
	print("Extracting parameters...")
	fullyFeaturedModel = getStates(main_stack[0], 'Atomic')

	# Convert and write JSON object to file
	with open("model_formalism.json", "w") as outfile:
		json.dump(fullyFeaturedModel, outfile, indent = 2)
	
	print(Fore.GREEN, "JSON created successfully")