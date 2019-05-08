#!/bin/sh
# Linux has a number of boot parameters that enhances CPU isolation:

#  isolcpus=<cpu set> This parameter specifies a set of CPUs that will be excluded from the Linux scheduler load balancing algorithm. The set is specified as a comma separated list of cpu numbers or ranges. E.g. "0", "1-2" or "0,3-4". The set specification must not contain any spaces. It is definitely recommended to use this parameter if the target kernel lacks support for CPU hotplug.
#  nohz_full=<cpu set> A list of CPUs for which full dynamic ticks should be enabled. If the kernel configuration CONFIG_NO_HZ_FULL_ALL was given, then this list will be all CPUs except CPU 0, and this boot option is not needed.

# To achieve isolation in the RT domain (CPU2 and CPU3), use the following parameters:
###
# isolcpus=2,3 nohz_full=2,3
###

# After the system has booted, check the boot messages to verify that full dynamic ticks was enabled, e.g. using the shell command dmesg. Search for entries similar to the following:
#  NO_HZ: Full dynticks CPUs: 2-3.

# Also make sure there is an entry similar to the following:
# Experimental no-CBs CPUs: 0-7.

# The no-CB CPU list must include the CPU list for full dynticks.

# When choosing the CPU lists on hardware using simulated CPUs, such as hyperthreads, ensure you include real cores and not half a core. The latter could occur if one hyperthread is in the set of CPUs using full dynamic ticks feature while the other hyperthread on the same core does not. This can cause problems when pinning interrupts to a CPU. The two hyperthreads might also affect each other depending on the load.

##### Generaly because our application is multi-threaded, full dynamic ticks is not encouraged.



