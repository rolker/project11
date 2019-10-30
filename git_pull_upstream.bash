#!/bin/bash

for d in ../catkin_ws/src/*; do
    if [[ -d $d ]]; then
        if [[ -d $d/.git ]]; then
            echo
            echo "*** $d ***"
            echo
            cd $d
            git pull upstream master
            cd ../../../scripts
        fi
    fi
done

echo
echo "*** documentation ***"
echo
cd ../documentation
git pull upstream master
cd ../scripts


echo
echo "*** scripts ***"
echo
git pull upstream master
