#!/bin/bash

for d in ../catkin_ws/src/*; do
    if [[ -d $d ]]; then
        if [[ -d $d/.git ]]; then
            echo
            echo "*** $d ***"
            echo
            cd $d
            git pull
            cd ../../../scripts
        fi
    fi
done

echo
echo "*** documentation ***"
echo
cd ../documentation
git pull
cd ../scripts


echo
echo "*** scripts ***"
echo
git pull
