#!/bin/bash

distancias=(60 90 110 140 170)
diretorios=(1euclideanexperiment 2euclideanexperiment 3euclideanexperiment 5euclideanexperiment)
#diretorios=(5euclideanexperiment)
experiments=(1 2 3 4 5)

#distancias=(60)
#diretorios=(1euclideanexperiment)
#experiments=(1)

for dir_ in ${diretorios[@]}; do
  export EXP_DIR=~/NetworkedRobotsProject/simulated_experiments/$dir_
  for dist in ${distancias[@]}; do
    for exp in ${experiments[@]}; do
	export LOG_DIR=$dir_$'/cefsm_final/cefsm_'$dist'/'
	#export LOG_DIR=$dir_$'_cefsm_'$dist'/'
	#export LOG_DIR=$dir_$'_cefsm_'$dist'/'
	./client/client.py -c $EXP_DIR/../config_sim.yaml &

	cd simulated_experiments/
	./create_config.py $EXP_DIR
	cd ../
	
	cd simulated_experiments/cefsm/
	./create_exp.py 20 $dist 0.05
	cd ../../

	roslaunch simulated_experiments/simulation.launch &

	sleep 5

	roslaunch simulated_experiments/cefsm/all.launch

    done
  done
done





#TODO: 
#Log
#numero de robos na solucao
#Coviriancia
