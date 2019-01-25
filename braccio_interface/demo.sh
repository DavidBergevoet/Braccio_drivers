#!/bin/sh

sleep 8
echo Moving to ready in 5 secs.

rosservice call /MoveToPosition "position: 'ready' 
duration: 5.0"

sleep 6

echo Moving to straight in 5 secs.

rosservice call /MoveToPosition "position: 'straight'
duration: 5.0"

sleep 4

echo Moving to park with emergency stop after 2 secs.

rosservice call /MoveToPosition "position: 'park'
duration: 6.0"

sleep 2

echo Emergencystop

rosservice call /s

sleep 3

echo Exit with the ready command

rosservice call /MoveToPosition "position: 'ready'
duration: 5.0"

sleep 4

pkill braccio_interface
