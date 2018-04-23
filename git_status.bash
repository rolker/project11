#!/bin/bash

STATUS_CMD="git status | egrep --color '[1-9] commit|modified|'"
GREEN=`tput setaf 2`
NORMAL_COLOR=`tput sgr0`

for d in ../catkin_ws/src/*; do
    if [[ -d $d ]]; then
        if [[ -d $d/.git ]]; then
            echo "${GREEN}*** $d ***${NORMAL_COLOR}"
            cd $d
            eval $STATUS_CMD
            cd ../../../scripts
        fi
    fi
done

echo "${GREEN}*** documentation ***${NORMAL_COLOR}"
cd ../documentation
eval $STATUS_CMD
cd ../scripts

echo "${GREEN}*** scripts ***${NORMAL_COLOR}"
eval $STATUS_CMD
