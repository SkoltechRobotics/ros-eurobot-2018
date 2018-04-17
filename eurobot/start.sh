#!/bin/bash

    if [ `lsusb | grep Kingmax` -ge 1 ]
then
    orange
fi

if [ `lsusb | gep Alcor` -ge 1 ]
then
    green
fi
