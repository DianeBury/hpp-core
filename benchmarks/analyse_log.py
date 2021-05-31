#!/usr/bin/env python

import os, sys
import glob
from util import pretty_dict_string
import pandas as pd

devel_dir = os.getenv ('DEVEL_HPP_DIR')

def countPatternBetweenStopper (filename, pattern, stopper):
    res = []
    count = 0
    with open (filename, 'r') as f:
        for line in f:
            if stopper in line:
                if count != 0:
                    res.append(count)
                count = 0
            elif pattern in line:
                count += 1
    if count != 0:
        res.append(count)
    return res

def all_same_value(dico):
    value = None
    for key in dico:
        if value is None:
            value = dico[key]
        elif value != dico[key]:
            return False
    return True

def filename_from_pid(pid):
    filename = devel_dir + '/install/var/log/hpp/benchmark.' + pid + '.log'
    return filename

cases = ['Basic', 'Sorting', 'Vmax', 'Memory', 'Complete']

def analyse(filename=None):
    patterns = {c:'computeDistanceLowerBound '+c for c in cases}
    stoppers = {c:c+' new path' for c in cases}
    counts = {c:countPatternBetweenStopper(filename, patterns[c], stoppers[c]) for c in cases}
    counts = {c:temps[c] for c in counts if len(counts[c])>0}
    stats = {c:pd.DataFrame(counts[c]) for c in counts}
    return counts, stats

if __name__ == '__main__':
    if len(sys.argv) <=1:
        print("Finding most recent log ...")
        list_of_files = glob.glob(devel_dir + '/install/var/log/hpp/*.log')
        if len(list_of_files) == 0:
            print("No log file in " + devel_dir + '/install/var/log/hpp/*.log')
        else:
            latest_file = max(list_of_files, key=os.path.getctime)
            pid = latest_file.split('benchmark.')[1].split('.log')[0]
            print("Latest benchmark log file found: " + pid)
            results, stats = analyse(filename_from_pid(pid))
            for c in stats:
                print(c)
                print(stats[c].describe())
                print(' ')
    else:
        filename = sys.argv[1]
        results = analyse(filename)
        