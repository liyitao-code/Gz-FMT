#!/bin/bash

for i in ../nuc/*
do
    if grep "#0.*Object" $i/gz.out 2> /dev/null 1> /dev/null
    then
        echo $i
        ./replay.sh $i
    fi
done
