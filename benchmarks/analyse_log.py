#!/usr/bin/env python

import os, sys
import glob
from util import pretty_dict_string
import pandas as pd

devel_dir = os.getenv ('DEVEL_HPP_DIR')

def countPatternBetweenStopper (pid, pattern, stopper):
    filename = devel_dir + '/install/var/log/hpp/benchmark.' + pid + '.log'
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
    res.append(count)
    return res

def get_number_of_call(pid):
    pass

def all_same_value(dico):
    value = None
    for key in dico:
        if value is None:
            value = dico[key]
        elif value != dico[key]:
            return False
    return True

cases = ['Basic', 'Sorting', 'Vmax', 'Memory', 'Complete']

def analyse(pid):
    patterns = {c:'computeDistanceLowerBound '+c for c in cases}
    stoppers = {c:c+' new path' for c in cases}
    temps = {c:countPatternBetweenStopper(pid, patterns[c], stoppers[c]) for c in cases}
    nb_of_paths = {c:len(temps[c]) for c in cases}
    if not all_same_value(nb_of_paths):
        print("Error, all methods don't have the same number of paths validated")
    Ntests = len(temps[cases[0]]) / 3
    counts = {}
    counts['ArmNeutral'] = {c:temps[c][0:Ntests] for c in temps}
    counts['FixedBase'] = {c:temps[c][Ntests:2*Ntests] for c in temps}
    counts['FullBody'] = {c:temps[c][2*Ntests:] for c in temps}
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
            results, stats = analyse(pid)
            for c in stats:
                print(c)
                print(stats[c].describe())
                print(' ')
    else:
        idx = sys.argv[1]
        results = analyse(str(idx))
        