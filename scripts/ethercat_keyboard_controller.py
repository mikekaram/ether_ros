#!/usr/bin/env python
from cmd import Cmd
import sys
import subprocess
import time

import rospy
from ighm_ros.srv import ModifyOutputBit, ModifyOutputSInt16, ModifyOutputSInt32, ModifyOutputUInt16, EthercatCommd, ModifyOutputSByte

help_message = """
    #########################################################################################################################
    ## Ethercat Keyboard controller for changing elliptic trajectory parameters  and controlling the ethercat communicator ##
    #########################################################################################################################

    All the terminal commands must beggin with an exclamation mark "!".
    If you want to find the current application variables, see the EthercatOutputData.msg
    The current supported commands are:

    !start : starts the ethercat communicator
    !stop : stops the ethercat communicator
    !restart : restarts the ethercat communicator
    !variable [slave_id | 'all'] [variable_name] [value] : change the value of a variable in the ethercat output data
    !run [script_to_run] : run the script specified, inside the ighm_ros/scripts directory
    !help : shows this help message
    !q : exit the terminal

    Type !q to quit
    """



class ethercat_controller(Cmd):

    variables2indeces = {
        "state_machine" : [[0, 0], "bit"],
        "initialize_clock": [[0, 1], "bit"],
        "initialize_angles": [[0, 2], "bit"],
        "inverse_kinematics": [[0, 3], "bit"],
        "blue_led": [[0, 4], "bit"],
        "red_led": [[0, 5], "bit"],
        "button_1": [[0, 6], "bit"],
        "button_2": [[0, 7], "bit"],
        "sync": [[1], "sbyte"],
        "desired_x_value": [[2], "sint32"],
        "filter_bandwidth": [[6], "uint16"],
        "desired_y_value": [[8], "sint32"],
        "kp_100_knee": [[12], "sint16"],
        "kd_1000_knee": [[14], "sint16"],
        "ki_100_knee": [[16], "sint16"],
        "kp_100_hip": [[18], "sint16"],
        "kd_1000_hip": [[20], "sint16"],
        "ki_100_hip": [[22], "sint16"],
        "x_cntr_traj1000": [[24], "sint16"],
        "y_cntr_traj1000": [[26], "sint16"],
        "a_ellipse100": [[28], "sint16"],
        "b_ellipse100": [[30], "sint16"],
        "traj_freq100": [[32], "sint16"],
        "phase_deg": [[34], "sint16"],
        "flatness_param100": [[36], "sint16"]
    }

    def call_modify_service(self, slave_id, variable_name, value):
        service_arguments_list = [slave_id]
        service_arguments_list.extend(
            self.variables2indeces[variable_name][0])
        try:
            service_arguments_list.append(int(value))
        except ValueError:
            service_arguments_list.append(float(value))
        self.call_service_mux(
            self.variables2indeces[variable_name][1], *service_arguments_list)
        del service_arguments_list[:]

    def call_service_mux(self, name, *data):
        function = self.function_dictionary[name]
        print function(self, *data)

    def modify_output_bit_client(self, slave_id, index, subindex, value):
        rospy.wait_for_service('modify_output_bit')
        try:
            modify_output_bit = rospy.ServiceProxy(
                'modify_output_bit', ModifyOutputBit)
            response = modify_output_bit(slave_id, index, subindex, value)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def modify_output_sbyte_client(self, slave_id, index, value):
        rospy.wait_for_service('modify_output_sbyte')
        try:
            modify_output_sbyte = rospy.ServiceProxy(
                'modify_output_sbyte', ModifyOutputSByte)
            response = modify_output_sbyte(slave_id, index, value)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def modify_output_sint16_client(self, slave_id, index, value):
        rospy.wait_for_service('modify_output_sint16')
        try:
            modify_output_sint16 = rospy.ServiceProxy(
                'modify_output_sint16', ModifyOutputSInt16)
            response = modify_output_sint16(slave_id, index, value)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def modify_output_uint16_client(self, slave_id, index, value):
        print(slave_id, index, value)
        rospy.wait_for_service('modify_output_uint16')
        try:
            modify_output_uint16 = rospy.ServiceProxy(
                'modify_output_uint16', ModifyOutputUInt16)
            response = modify_output_uint16(slave_id, index, value)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def modify_output_sint32_client(self, slave_id, index, value):
        rospy.wait_for_service('modify_output_sint32')
        try:
            modify_output_sint32 = rospy.ServiceProxy(
                'modify_output_sint32', ModifyOutputSInt32)
            response = modify_output_sint32(slave_id, index, value)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def ethercat_communicator_client(self,mode):
        rospy.wait_for_service('ethercat_communicatord')
        try:
            ethercat_communicator = rospy.ServiceProxy(
                'ethercat_communicatord', EthercatCommd)
            response = ethercat_communicator(mode)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def do_shell(self, args):
        print(args)
        arguments = args.split(" ")
        if "start" in arguments:
            if len(arguments) != 1:
                print "Usage: !start"
            else:
                self.call_service_mux("ethercat_communicator", "start")
        elif "stop" in arguments:
            if len(arguments) != 1:
                print "Usage: !stop"
            else:
                self.call_service_mux("ethercat_communicator", "stop")
        elif "restart" in arguments:
            if len(arguments) != 1:
                print "Usage: !restart"
            else:
                self.call_service_mux("ethercat_communicator", "restart")
        elif "variable" in arguments:
            if len(arguments) != 4:
                print "Usage: !variable [slave_id | 'all'] [variable_name] [value]"
            else:
                if(arguments[1] == "all"):
                    while(not rospy.has_param('/ethercat_slaves/slaves_count')):
                        print("Waiting for 'slaves_count' variable to be set.")
                        time.sleep(1)
                    slaves_count = rospy.get_param(
                        '/ethercat_slaves/slaves_count')
                    for i in range(slaves_count):
                        self.call_modify_service(
                            i, arguments[2], arguments[3])
                else:
                    self.call_modify_service(
                        int(arguments[1]), arguments[2],  arguments[3])
        elif "run" in arguments:
            if len(arguments) != 2:
                print "Usage: !run [script_to_run]"
            else:
                arguments[1] = arguments[1].replace("/","")
                arguments[1].strip()
                p = subprocess.Popen('~/catkin_ws/src/ighm_ros/scripts/'+arguments[1], shell=True)
                p.wait()
        elif "help" in arguments:
            print(help_message)
        elif "q" in arguments:
            exit(0)
        else:
            self.default(args)

    def do_help(self, line):
        print(help_message)

    def default(self, line):
        print("Unrecognized command. Please type '!help' or '?' for help.")

    @classmethod
    def emptyline(cls):
        pass
    function_dictionary = {"bit": modify_output_bit_client,
                           "sint16": modify_output_sint16_client,
                           "uint16": modify_output_uint16_client,
                           "sint32": modify_output_sint32_client,
						   "sbyte" : modify_output_sbyte_client,
                           "ethercat_communicator": ethercat_communicator_client}

if __name__ == '__main__':

    intro_message = "Welcome to the Ethercat Keyboard Controller!\nBy Mike Karamousadakis\nContact: mkaramousadakis@zoho.eu\n"
    print(intro_message)
    print(help_message)
    prompt = ethercat_controller()
    prompt.prompt = '[{}]> '.format("ethercat controller")
    prompt.intro = help_message
    prompt.cmdloop('Initializing the controller...\n')
