#!/usr/bin/env python

import sys
import pandas as pd
import os
trace_file_path = "~/catkin_ws/src/ether_ros/scripts/trace_cmd/"
trace_file_name = sys.argv[1]
csv_file_name = sys.argv[1].replace(".txt",".csv")
print("Name of the output csv file: " + csv_file_name)
with open(os.path.abspath(trace_file_name),'r') as trace_data:
    pass


if __name__ == "__main__":
    pass