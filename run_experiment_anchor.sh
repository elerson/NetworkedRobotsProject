#!/bin/bash

distancias=(90 110 140 170)
diretorios=(1euclideanexperiment 2euclideanexperiment 3euclideanexperiment 5euclideanexperiment)
#diretorios=(1euclideanexperiment)
experiments=(1)

for dir_ in ${diretorios[@]}; do
  export EXP_DIR=~/NetworkedRobotsProject/simulated_experiments/$dir_
  for dist in ${distancias[@]}; do
    for exp in ${experiments[@]}; do
	export LOG_DIR=$dir_$'/anchor_final/anchor_'$dist'/'

	./client_anchor/client.py -c $EXP_DIR/../config_sim.yaml -r $dist &
	
	cd simulated_experiments/
	./create_config.py $EXP_DIR
	cd ../

	cd simulated_experiments/anchor/
	./create_exp.py 10 $dist 0.05
	cd ../../

	roslaunch simulated_experiments/simulation.launch &

	sleep 5

	roslaunch simulated_experiments/anchor/all_terminals.launch &

	sleep 5
	roslaunch simulated_experiments/anchor/all_robots.launch

	killall roslaunch
	sleep 100

    done
  done
done





#TODO: 
#Log
#numero de robos na solucao
#Coviriancia
