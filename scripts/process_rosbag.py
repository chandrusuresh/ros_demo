import os
import matplotlib.pyplot as plt
import numpy as np
import importlib
importlib.import_module("ros_readbagfile")
from ros_readbagfile import parseArgs,printMsgsInBagFile

import rosbag

def readIMU(data,msg):
    w   = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    acc = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
    if data == None:
        data = dict()
        data["angular_velocity"]    = w
        data["linear_acceleration"] = acc
    else:
        data["angular_velocity"]    = np.c_[data["angular_velocity"],w]
        data["linear_acceleration"] = np.c_[data["linear_acceleration"],acc]
    return data

def readMsgsInBagFile(args):
    no_msgs_found = True
    # Establish counters to keep track of the message number being received under each topic name
    msg_counters = {} # empty dict
    total_count = 0
    bag_in = rosbag.Bag(args.bag_file_in)
    bag_data = dict()
    for topic, msg, t in bag_in.read_messages(args.topics_to_read):
        if not ("imu" in topic or "imu" in msg):
            continue
        
        total_count += 1
        no_msgs_found = False
        # Keep track of individual message counters for each message type
        if topic not in msg_counters:
            msg_counters[topic] = 1;
        else:
            msg_counters[topic] += 1;

        if topic not in bag_data:
            bag_data[topic] = None
        
        bag_data[topic] = readIMU(bag_data[topic],msg)

    print("# Total messages found: {:>16}".format(total_count))
    print("#")
    for topic in msg_counters:
        print("#    {:<30} {:>4}".format(topic + ":", msg_counters[topic]))
        print(topic + ":",bag_data[topic]["angular_velocity"].shape)
    if no_msgs_found:
        print("# NO MESSAGES FOUND IN THESE TOPICS")
    print("#")
    print("# DONE.")
    print("# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    return bag_data

def plot_data(data,saveFig=False):
    topics = data.keys()
    msgs = data[topics[0]].keys()
    print(msgs)
    rows = data[topics[0]][msgs[0]].shape[0]
    for m in msgs:
        fig,ax = plt.subplots(rows,1,figsize=(10,7))
        for t in topics:
            d = data[t][m]
            for r in range(d.shape[0]):
                ax[r].plot(d[r,:])
        plt.legend(topics)
        plt.suptitle(m)
        if (saveFig):
            savePath = os.path.join(os.getcwd(),"plots")
            if not os.path.exists(savePath):
                os.mkdir(savePath)
            plt.savefig(os.path.join(savePath,m+".jpg"))
        plt.show()
        # plt.clf()

if __name__ == '__main__':
    args = parseArgs()
    # printMsgsInBagFile(args)
    bag_data = readMsgsInBagFile(args)
    plot_data(bag_data,True)