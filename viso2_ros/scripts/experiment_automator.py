#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import subprocess
import time
import os
import yaml
import argparse
from pathlib import Path


def node_killer(node):

    os.system("rosnode kill " + node)


def rostopic_creator(t, loc, ver, path, i):

    if "odometry" in t:
        file_name = "trajectoria"

    elif "info" in t:
        file_name = "info" 

    file_name = file_name + loc + ver + "_" + str(i) + ".txt"

    print("rostopic echo -p " + t + " > " + file_name)

    subprocess.Popen("rostopic echo -p " + t + " > " + file_name, shell=True, cwd = path)


def experiment_automator(output_path, location, upper_limit):

    versions = {'ORB_tracker': 0,
                'LIBVISO2': 0, 
                'SIFT_SIFT': 1, 
                'SIFT_SIFTNoBuck': 1, 
                'SURF_SIFT': 2, 
                'SURF_SIFTNoBuck': 2, 
                'SURF_BRISK': 3,
                'SURF_BRISKNoBuck': 3,
                'SURF_FREAK': 4,
                'SURF_FREAKNoBuck': 4}
    
    # versions = {'LIBVISO2': 0, 
    #             'SIFT_SIFT': 1, 
    #             'SIFT_SIFTNoBuck': 1, 
    #             'SURF_SIFT': 2, 
    #             'SURF_SIFTNoBuck': 2, 
    #             'SURF_BRISK': 3,
    #             'SURF_BRISKNoBuck': 3,
    #             'SURF_FREAK': 4,
    #             'SURF_FREAKNoBuck': 4}

    with open('/home/uib/catkin_ws/src/viso2/viso2_ros/config/viso2_stereo_parameters_auto.yaml', 'r') as file:
        configs = yaml.safe_load(file)

    for version, version_value in versions.items():

        print("Version: ", version)

        for tag1, value1 in configs['stereo_odometer'].items():

            if tag1 == "detection_and_tracking_version":
                configs['stereo_odometer'][tag1] = version_value

            if tag1 == "enable_bucketing":

                if "NoBuck" in version:
                    configs['stereo_odometer'][tag1] = False

                else:
                    configs['stereo_odometer'][tag1] = True

        with open('/home/uib/catkin_ws/src/viso2/viso2_ros/config/viso2_stereo_parameters_auto.yaml', 'w') as file:
            yaml.dump(configs, file)

        version_result_storage = Path(os.path.join(output_path, version))
        if not version_result_storage.exists():
            os.makedirs(version_result_storage) 
            print(f"Directory {version} created in {version_result_storage}")
            lower_limit = 0

        else:
            lower_limit = int(len(os.listdir(version_result_storage)) / 2)
            
        print("Lower limit: ", lower_limit)
        print("Upper limit: ", upper_limit)   

        for iter in range(lower_limit, upper_limit, 1):
            print(iter)

            if "ORB_tracker" in version:
                namespace = "/orb_tracker"
            
            else:
                namespace = "/stereo_odometer"

            rostopic_creator(namespace + "/odometry", location, version, version_result_storage, iter)
            rostopic_creator(namespace + "/info", location, version, version_result_storage, iter)

            print("Topics created")

            time.sleep(30)

            still_running = True
            
            if "ORB_tracker" in version:
                subprocess.Popen("roslaunch orb_tracker orb_tracker_auto.launch", shell=True)
            
            else:
                subprocess.Popen("roslaunch viso2_ros stereo_odometer_auto.launch", shell=True)

            while still_running:
                print("I'm going tu sleep!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                time.sleep(10)
                info_dict = yaml.safe_load(subprocess.Popen(["rosnode list"], shell=True, stdout = subprocess.PIPE).communicate()[0])
                info_dict = info_dict.split(" ")
                print("Number of nodes: ", len(info_dict))
                
                if len(info_dict) <= 6:
                    still_running = False

            print("The odometer finished his work!!")

            nodes = yaml.safe_load(subprocess.Popen(["rosnode list"], shell=True, stdout = subprocess.PIPE).communicate()[0])
            nodes = nodes.split(" ")
            print("Number of nodes: ", len(nodes))
            print("Nodes: ", nodes)

            for node in nodes:
                if "rostopic_" in node:
                    node_killer(node)
                    time.sleep(1)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="")
    parser.add_argument("--out_path", default = os.getcwd(), help = "Path ot save results")
    parser.add_argument("--location", default = "Portals", help = "Phisical location of the mission")
    parser.add_argument("--num_experiment", default = 10, help = "Number of experiments per version") # 20
    args = parser.parse_args()

    experiment_automator(args.out_path, args.location, args.num_experiment)
    print("Finished cleanly")




