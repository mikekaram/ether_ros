#!/usr/bin/env python

import sys
import pandas as pd
import os
trace_file_path = "/home/mikekaram/catkin_ws/src/ether_ros/experiments/17May2019/"
trace_file_name = sys.argv[1]
csv_file_name = sys.argv[1].replace(".txt",".csv")
print("Name of the output csv file: " + csv_file_name)

function_stack = []
output_csv_list = []
with open(os.path.abspath(trace_file_path+trace_file_name), 'r') as trace_data:
    lines = trace_data.readlines()
for line in lines:
    line_list = line.split()
    # discard lines with not proper data inside (e.g. first 2 lines)
    if(len(line_list) > 5):
        if("funcgraph" in line_list[3]):
            if(line_list[3] == "funcgraph_entry:"):
                line_list[3] = "funcgraph"
                if("()" in line_list[5]):
                    line_list[4] = line_list[5]
                    output_csv_list.append(line_list[0:6])
                    csv_index = len(output_csv_list) - 1
                    function_stack.append(csv_index)
                elif("()" in line_list[7]):
                    line_list[7] = line_list[7].strip(";")
                    line_list[5] = line_list[4]
                    line_list[4] = line_list[7]
                    output_csv_list.append(line_list[0:6])
                elif("()" in line_list[8]):
                    line_list[8] = line_list[8].strip(";")
                    line_list[4] = line_list[8]
                    output_csv_list.append(line_list[0:6])
                else:
                    print("Erroneous input: {}".format(line_list))
            else:
                funcgraph_entry = function_stack.pop()
                if("!" in line_list[4] or "+" in line_list[4]):
                    output_csv_list[funcgraph_entry][5] = line_list[5]
                else:
                    output_csv_list[funcgraph_entry][5] = line_list[4]
        if("sched" in line_list[3]):
            if(line_list[3] == "sched_wakeup:"):
                line_list[3] = "schedwakeup"
                line_list[0] = line_list[4]
                line_list[4] = "-"
                line_list[5] = "-"
                output_csv_list.append(line_list[0:6])
            else:
                print("Unsupported event:{}".format(line_list))

print(output_csv_list[:10])
columns = ["Process","CPU","Time", "TypeOfTrace", "Function", "Duration"]
df = pd.DataFrame(output_csv_list,columns=columns)
df['Time'] = df['Time'].str.strip(":")
df['Time'] = df['Time'].str.replace(".","")
print(df)
with open(os.path.abspath(trace_file_path+csv_file_name), 'w'):
    df.to_csv(trace_file_path+csv_file_name, index_label='Index')

if __name__ == "__main__":
    pass
