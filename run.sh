#!/bin/bash

path='/home/chunibyo/workspace/dataset/euroc/MH_01_easy'
traj='MH_01'

time ./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml "$path" ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt "$traj"
