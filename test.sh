#!/bin/bash

path='/home/chunibyo/workspace/dataset/euroc/MH_01_easy'
traj='MH_01'

rm results/"$traj".*

evo_ape tum "$path"/mav0/state_groundtruth_estimate0/data.tum kf_"$traj".txt -vpa --save_results results/"$traj".zip

evo_res results/"$traj".zip -p --save_table results/"$traj".csv