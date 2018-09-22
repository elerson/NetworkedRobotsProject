#!/bin/bash

distancias=(60 90)
diretorios=(1euclideanexperiment 2euclideanexperiment 3euclideanexperiment 5euclideanexperiment)
#diretorios=(5euclideanexperiment)
experiments=(1)

#distancias=(160)
#diretorios=(1euclideanexperiment 2euclideanexperiment 3euclideanexperiment 4euclideanexperiment 5euclideanexperiment)
#diretorios=(2euclideanexperiment)
#experiments=(1)


for dir_ in ${diretorios[@]}; do
  export EXP_DIR=~/NetworkedRobotsProject/simulated_experiments/$dir_
  for dist in ${distancias[@]}; do
    for exp in ${experiments[@]}; do
	export LOG_DIR=$dir_$'/gradient_final/gradient_'$dist'/'
	#export LOG_DIR=$dir_$'_cefsm_'$dist'/'
	./client/client.py -c $EXP_DIR/../config_sim.yaml &
	
 	#cd simulated_experiments/cefsm/
	#./create_exp.py 7 ${distancias[d]} 0.05
	#cd ../../
	
	cd simulated_experiments/
	./create_config.py $EXP_DIR
	cd ../

	cd simulated_experiments/gradient/
	./create_exp.py 15 $dist 0.05
	cd ../../

	#./simulated_experiments/gradient/create_exp.py 25 ${distancias[d]} 0.05

	roslaunch simulated_experiments/simulation.launch &

	sleep 5

	#roslaunch simulated_experiments/cefsm/all.launch
	roslaunch simulated_experiments/gradient/all.launch

    done
  done
done





#TODO: 
#Log
#numero de robos na solucao
#Coviriancia
