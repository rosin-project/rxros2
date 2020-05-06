#!/bin/bash

echo "TEST: ros2_cpptest"

ros2 run ros2_cpptest1 ros2_t1 > /tmp/ros2_cpptest1_1M.txt &
ros2 run ros2_cpptest2 ros2_t2 > /tmp/ros2_cpptest2_1M.txt &

sleep 20

pid1=`ps | grep ros2_t1 | grep -v grep | awk '{print $1}'`
pid2=`ps | grep ros2_t2 | grep -v grep | awk '{print $1}'`

cat "/proc/$pid1/status" > /tmp/ros2_cpptest1_1M.status
cat "/proc/$pid2/status" > /tmp/ros2_cpptest2_1M.status

kill $pid1
kill $pid2

cat /tmp/ros2_cpptest1_1M.txt /tmp/ros2_cpptest2_1M.txt | sed -e 's/.*,//' | sort -n > /tmp/ros2_cpptest_1M.lat

echo "TEST: rxros2_cpptest"

ros2 run rxros2_cpptest1 rxros2_t1 > /tmp/rxros2_cpptest1_1M.txt &
ros2 run rxros2_cpptest2 rxros2_t2 > /tmp/rxros2_cpptest2_1M.txt &

sleep 20

pid1=`ps | grep rxros2_t1 | grep -v grep | awk '{print $1}'`
pid2=`ps | grep rxros2_t2 | grep -v grep | awk '{print $1}'`

cat "/proc/$pid1/status" > /tmp/rxros2_cpptest1_1M.status
cat "/proc/$pid2/status" > /tmp/rxros2_cpptest2_1M.status

kill $pid1
kill $pid2

cat /tmp/rxros2_cpptest1_1M.txt /tmp/rxros2_cpptest2_1M.txt | sed -e 's/.*,//' | sort -n > /tmp/rxros2_cpptest_1M.lat

echo "TEST: ros2_pytest"

ros2 run ros2_pytest1 ros2_t1 > /tmp/ros2_pytest1_1M.txt &
ros2 run ros2_pytest2 ros2_t2 > /tmp/ros2_pytest2_1M.txt &

sleep 20

pid1=`ps | grep ros2_t1 | grep -v grep | awk '{print $1}'`
pid2=`ps | grep ros2_t2 | grep -v grep | awk '{print $1}'`

cat "/proc/$pid1/status" > /tmp/ros2_pytest1_1M.status
cat "/proc/$pid2/status" > /tmp/ros2_pytest2_1M.status

kill $pid1
kill $pid2

cat /tmp/ros2_pytest1_1M.txt /tmp/ros2_pytest2_1M.txt | sed -e 's/.*,//' | sort -n > /tmp/ros2_pytest_1M.lat

echo "TEST: rxros2_pytest"

ros2 run rxros2_pytest1 rxros2_t1 > /tmp/rxros2_pytest1_1M.txt &
ros2 run rxros2_pytest2 rxros2_t2 > /tmp/rxros2_pytest2_1M.txt &

sleep 20

pid1=`ps | grep rxros2_t1 | grep -v grep | awk '{print $1}'`
pid2=`ps | grep rxros2_t2 | grep -v grep | awk '{print $1}'`

cat "/proc/$pid1/status" > /tmp/rxros2_pytest1_1M.status
cat "/proc/$pid2/status" > /tmp/rxros2_pytest2_1M.status

kill $pid1
kill $pid2

cat /tmp/rxros2_pytest1_1M.txt /tmp/rxros2_pytest2_1M.txt | sed -e 's/.*,//' | sort -n > /tmp/rxros2_pytest_1M.lat

exit 0
