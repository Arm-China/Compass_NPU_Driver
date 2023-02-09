#!/bin/bash

KMD_SRC_PATH=./

function check_in_dir(){
    for file in `ls $1`
    do
        dir_or_file=$1"/"$file
        if [ -d $dir_or_file ]; then
            check_in_dir $dir_or_file
        else
            if [ "${file##*.}" = "c" ] || [ "${file##*.}" = "h" ]; then
                ./checkpatch.pl --no-tree --strict -f $dir_or_file
            fi
        fi
    done
}

cd $KMD_SRC_PATH
make clean
cd -

echo "#################### KMD Source Coding Style Checking ######################"
echo "Checking Script(s): checkpatch.pl, Linux kernel 5.x"
check_in_dir $KMD_SRC_PATH
echo "############################################################################"
