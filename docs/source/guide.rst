Guide
======
This guide will solve your problem of where to start with documentation,
by providing a basic explanation of how to do it easily.
Look how easy it is to use:

- After you *catkin_make* the project, in one terminal run:

.. highlight:: bash
$ roslaunch ighm_ros ighm_ros.launch

- After that, and while the process is running, you run in another terminal:

.. highlight:: bash
$ rosrun ighm_ros ethercat_keyboard_controller.py


- Now you can give orders to the EtherCAT Communicator via a custom terminal. 
Have fun playing around!

- *Tip: You could run a bash script in the custom terminal by running:*

.. highlight:: bash
[ethercat_controller] > !r my_awesome_bash_script.sh

Notice that your script must be under the *scripts* directory. You could also check some 
example scripts there.

Features
--------

- Real time characteristics, using PREEMPT_RT patch
- EtherCAT technology adaptation in Linux
- Works in a recent version of ROS (kinetic)
- Utilizes the main development framework for EtherCAT applications, IgH Master kernel module

Installation
------------

More to write here.
Stay tuned. 
It will be intense.

Contribute
----------

- Issue Tracker: https://github.com/mikekaram/IgHMaster_userspace_program_in_ROS/issues
- Source Code: https://github.com/mikekaram/IgHMaster_userspace_program_in_ROS

Limitations / Steps Forward
-----------------------------

This program assumes that the actual control code of the robot is running in the EtherCAT slaves.
Therefore there is no connection between this program and ros_control, although the intention of the author
is to make this connection happen, for robots that do have a control api inside ROS. This of course means 
that the ros_control module should communicate afterwards with this program, to send new data to the EtherCAT
slaves. Needless to say, the EtherCAT slaves will have a much more passive role in this configuration.

Support
-------

If you are having issues, please let us know.
We don't have a mailing list yet, so the default way is by communicating with: mkaramousadakis@zoho.eu

License
-------
The project is licensed under the GPLv2 licence. See more details in the source files of the project or in
the Lincence section.