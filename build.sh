#!/bin/bash

if [ "$1" = "all" ]; then

    echo "Configuring and building Thirdparty/DBoW2 ..."

    cd Thirdparty/DBoW2
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j

    cd ../../g2o

    echo "Configuring and building Thirdparty/g2o ..."

    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j

    cd ../../../
fi

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3
