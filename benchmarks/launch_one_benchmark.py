#!/usr/bin/env python

import os, sys
import benchmark_continuous_validation as benchmark
import analyse_log
from util import pretty_print_stats

devel_dir = os.getenv ('DEVEL_HPP_DIR')

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Argument should be one of the following: Complete, Basic, Sorting, Vmax, Memory")
    else:
        method = sys.argv[1]
        if method.lower() not in ['complete', 'basic', 'sorting', 'vmax', 'memory']:
            print("Argument should be one of the following: Complete, Basic, Sorting, Vmax, Memory")
        else:
            benchmark.run(method)
            # Finding the file log and moving it
            list_of_files = glob.glob(devel_dir + '/install/var/log/hpp/*.log')
            if len(list_of_files) == 0:
                print("No log file in " + devel_dir + '/install/var/log/hpp/*.log')
                print("Is logging activated ?")
            else:
                latest_file = max(list_of_files, key=os.path.getctime)
                new_filename = devel_dir + '/src/agimus-demos/tiago/deburring/data/benchmark_' + method.lower() + '.log'
                print("Moving log file (" + latest_file +") to data folder (" + new_filename + ")")
                os.rename(latest_file, new_filename)
                counts, stats = analyse_log.analyse(new_filename)
                pretty_print_stats(stats)