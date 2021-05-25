#!/bin/bash

list="basic memory sorting vmax complete"
[[ $list =~ (^|[[:space:]])"$1"($|[[:space:]]) ]] && echo "Benchmark for $1 method" || { echo 'Argument must be a validation continuous method name'; exit 1; }

# terminator -e hppcorbaserver -p hold
# sleep 0.5
# terminator -e gepetto-gui -p hold
# terminator -e benchmark_travaling_salesman_$1.py

echo "Moving latest log file: $logfilename"
logfilename=$(ls $DEVEL_HPP_DIR/install/var/log/hpp/ -t1 | head -n 1)
echo "$DEVEL_HPP_DIR/install/var/log/hpp/$logfilename"
mv "$DEVEL_HPP_DIR/install/var/log/hpp/$logfilename" "${DEVEL_HPP_DIR}"/src/agimus-demos/tiago/deburring/data/benchmark_"${1}.log"
echo "Move done"
newlogfilename="${DEVEL_HPP_DIR}/src/agimus-demos/tiago/deburring/data/benchmark_${1}.log"
echo "New file name : $newlogfilename"
terminator -e "cd $DEVEL_HPP_DIR/src/hpp-core/benchmarks && ipython -i analyse_log.py $newlogfilename" -p hold
