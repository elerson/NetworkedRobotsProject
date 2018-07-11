#!/bin/bash

distancias=(60 90 110 140 170)
diretorios=(1euclideanexperiment 2euclideanexperiment 3euclideanexperiment 4euclideanexperiment 5euclideanexperiment)
#diretorios=(5euclideanexperiment)
experiments=(1 2 3 4 5)


for dir_ in ${diretorios[@]}; do
  export EXP_DIR=~/NetworkedRobotsProject/simulated_experiments/$dir_
  for dist in ${distancias[@]}; do
    for exp in ${experiments[@]}; do
	export LOG_DIR=$dir_$'/gradient/gradient_'$dist'/'
	#export LOG_DIR=$dir_$'_cefsm_'$dist'/'
	./client/client.py &
	
 	#cd simulated_experiments/cefsm/
	#./create_exp.py 7 ${distancias[d]} 0.05
	#cd ../../


	cd simulated_experiments/gradient/
	./create_exp.py 20 $dist 0.05
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
