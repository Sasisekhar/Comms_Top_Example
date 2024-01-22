import html
from diagrams import Diagram, Cluster, Edge, Node
from diagrams.c4 import Person, Container, Database, System, SystemBoundary, Relationship
import json

def createCoupledCluster(atomics, name, instances, IC, EIC, EOC):
	result = {}
	flag = False
	EIC_nodes = {}
	EOC_nodes = {}
	ACI = {}
	ACO = {}

	with Cluster(name):
		for atomic in atomics:
			if "Coupled" in atomic.keys():
				ACI, ACO = createCoupledCluster(atomic['Atomics'], atomic['Coupled'].split('/')[-1].split('.')[0], atomic["Instance_names"], atomic['IC'], atomic['EIC'], atomic['EOC'])
			else:
				flag = True
				atomic_name = atomic['Atomic'].split('/')[-1].split('.')[0]
				
				instance_name = instances[atomic_name]
				instance_name_escape = html.escape(instance_name)
				atomic_name_escape = html.escape(atomic_name)
				title = f'<<font point-size="12"><b>{instance_name_escape}</b></font><br/><font point-size="9">[{atomic_name_escape}]</font><br/><br/><br/>>'
				atomic_node = Node(
					label = title
				)
				result.update({instance_name : atomic_node})
		
		if flag:
			from_models = list(IC.keys())
			for i in from_models:
				if(i in EIC):
					EIC_nodes.update({i : [IC[i], result[IC[i].split('->')[0]]]})
				elif(IC[i] in EOC):
					EOC_nodes.update({IC[i] : [i, result[i.split('->')[0]]]})
				else:
					if(IC[i].split('->')[-1].strip() in list(ACI.keys())):
						result[i.split('->')[0]] >> Edge(label = i.split('->')[-1] + ':' + ACI[IC[i].split('->')[-1].strip()][0].split('->')[-1].strip()) >> ACI[IC[i].split('->')[-1].strip()][1]
					elif(i.split('->')[-1].strip() in list(ACO.keys())):
						ACO[i.split('->')[-1].strip()][1] >> Edge(label = (ACO[i.split('->')[-1].strip()][0].split('->')[-1].strip() + ":" + IC[i].split('->')[-1])) >> result[IC[i].split('->')[0]]
					else:
						result[i.split('->')[0]] >> Edge(label = (i.split('->')[-1] + ':' + IC[i].split('->')[-1])) >> result[IC[i].split('->')[0]]
	return [EIC_nodes, EOC_nodes]

if __name__ == '__main__':

	f = open('model_formalism.json')

	data = json.load(f)

	f.close()

	with Diagram("Model Graph", show = True):
			top = data['Top'][0]
			top_name = top['Coupled'].split('/')[-1].split('.')[0]
			
			createCoupledCluster(top["Atomics"], top_name, top['Instance_names'], top['IC'], top['EIC'], top['EOC'])

