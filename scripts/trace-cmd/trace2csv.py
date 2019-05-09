#!/usr/bin/env python

import sys
import pandas as pd
import os
trace_file_path = "~/catkin_ws/src/ether_ros/scripts/trace_cmd/"
trace_file_name = sys.argv[1]
csv_file_name = sys.argv[1].replace(".txt",".csv")
print("Name of the output csv file: " + csv_file_name)

function_stack = []
with open(os.path.abspath(trace_file_name), 'r') as trace_data:
    lines = trace_data.readlines()
for index, line in enumerate(lines):
    line = line.split()
    # discard lines with not proper data inside (e.g. first 2 lines)
    if(len(line) > 5):
        if("funcgraph" in line[3]):
            if(line[3] == "funcgraph_entry:"):
                line[3] = "funcgraph"
                if("()" in line[5]):
                    line[4] = line[5]
                    function_stack.append(index)
                elif("()" in line[7]):
                    line[7] = line[7].strip(";")
                    line[5] = line[4]
                    line[4] = line[7]
                    del line[7]
                    del line[6]
                else:
                    print("Erroneous input: {}", line)
            else:
                funcgraph_entry = function_stack.pop()
                print (funcgraph_entry)
                if("!" in line[4] or "+" in line[4]):
                    print(lines[funcgraph_entry])
                    lines[funcgraph_entry][5] = line[5]
                else:
                    print(lines[funcgraph_entry])
                    lines[funcgraph_entry][5] = line[4]
                del lines[index]
        else:
            del lines[index]
df = pd.DataFrame(lines)
# df = pd.DataFrame([x.split() for x in lines if(len(x.split()) > 5)])
# df[2] = df[2].str.strip(":")
# df[3] = df[3].str.strip(":")
# print(df[3])
print(df)


if __name__ == "__main__":
    pass
