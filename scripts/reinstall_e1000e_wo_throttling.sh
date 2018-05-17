#!/bin/bash
rmmod e1000e
modprobe e1000e InterruptThrottleRate=0 RxIntDelay=0 TxIntDelay=0
