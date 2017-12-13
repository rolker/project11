#!/bin/bash

for d in ../catkin_ws/src/*; do
    if [[ -d $d ]]; then
        if [[ -d $d/.git ]]; then
            echo
            echo "*** $d ***"
            echo
            cd $d
            git status
            cd ../../../scripts
        fi
    fi
done

echo
echo "*** documentation ***"
echo
cd ../documentation
git status
cd ../scripts

echo
echo "*** scripts ***"
echo
git status
