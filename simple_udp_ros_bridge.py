"""
Simple system to bridge UDP packets from/to ROS

http://ricardodeazambuja.com

"""

import socket
from multiprocessing import Pipe, Process

import argparse

import rospy
import std_msgs.msg

class UDP2ROS(object):

    def __init__(self, ip_in, port_in, ip_out, port_out,
                 topic_to_publish_to, topic_to_subscribe_to,
                 msg_type="String", max_freq=10, buffer_size=1024, clean_loop=1):
        """
        Args:
            ip_in (str): IP adress to receive UDP packets.
            port_in (int): port number to receive UDP packets.
            ip_out (str): IP adress to send UDP packets.
            port_out (int): port number to send UDP packets.
            topic_to_publish_to (str): ROS topic name to publish received UDP packets.
            topic_to_subscribe_to (str): ROS topic name to subscribe to send UDP packets.
            msg_type (str): ROS std_msgs.msg
            max_freq (int): max frequency to publish.
            buffer_size (int): socket receiver buffer size (bytes).
            clean_loop (bool): clean the buffer before start.
        """
        # Pipe used to RECEIVE spikes FROM the UDP port
        pipe_in_r, pipe_in_w = Pipe(duplex=False)

        # Pipe used to SEND spikes TO the UDP port
        pipe_out_r, pipe_out_w = Pipe(duplex=False)

        processes = list()

        processes.append(Process(target=self.receive_UDP, args=(ip_in, port_in, pipe_in_w, buffer_size, clean_loop = True)))
        processes.append(Process(target=self.send_UDP, args=(ip_out, port_out, pipe_out_r)))
        processes.append(Process(target=self.receive_from_topic, args=(topic_to_subscribe_to, pipe_out_w, msg_type)))
        processes.append(Process(target=self.publish_to_topic, args=(topic_to_publish_to, pipe_in_r, msg_type, max_freq)))

        for p in processes:
            p.daemon = True # Guarantees the process will die after the main python

        self.processes = processes


    def start(self):
        """Start the processes and spins forever!
        """
        for p in self.processes:
            p.start()

        rospy.spin()


    @staticmethod
    def receive_UDP(ip_in, port_in, pipe_out, buffer_size, clean_loop = True):
        """This function simply creates a socket, reads all the UDP packets as they arrive and redirects to a multiprocessing.Pipe.
        Args:
            ip_in (str): "X.X.X.X" ordinary IP address from one of the network interfaces.
            port_in (int): 0 to 65535 (but you need to choose a free one).
            pipe_out (multiprocessing.Pipe): Pipe used to send the information received through UDP.
            buffer_size (int): Maximum amount of data to be received at once (in bytes).
        """

        sockI = socket.socket(socket.AF_INET,    # IP
                              socket.SOCK_DGRAM) # UDP

        sockI.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Tells the OS that if someone else is using the PORT, it
                                                                    # can use the same PORT without any error/warning msg.
                                                                    # Actually this is useful because if you restart the simulation
                                                                    # the OS is not going to release the socket so fast and an error
                                                                    # could occur.

        sockI.bind((ip_in, port_in)) # Bind the socket to the IPI/PORTI

        # Add here a test to check if the user asked to clean the buffer before start.
        while clean_loop:
            print "Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI
            try:
                data = sockI.recv(1, socket.MSG_DONTWAIT) # buffer size is 1 byte, NON blocking.
            except IOError: # The try and except are necessary because the recv raises a error when no data is received
                clean_loop = 0
        print "Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI, "...Done!"

        sockI.setblocking(1) # Tells the system that the socket recv() method will DO block until a packet is received

        # Until this point, the code is going to be executed only once each time the system runs.

        while True:
            try:
                pipe_out.send(sockI.recv(buffer_size)) # This is a blocking command, therefore the while loop is not going
                                                       # to eat up all the processor time.

            except IOError:  # Without the IOError even the keyboard "control+C" is caught here!
                print "UDP read error?"


            except ValueError:
                print "ValueError:", data  # DEBUG!
                #the data is corrupted, a wrong packet appeared at the port, etc...

            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # kills the while

    @staticmethod
    def send_UDP(ip_out, port_out, pipe_in):
        """This is a very light weight function to send information received through a multiprocessing Pipe using UDP.
        Args:
            ip_out (str): IP adress receiving the msg.
            port_out (int): port number receiving the msg.
            pipe_in (multiprocessing.Pipe): Pipe where the msg will receive.
        """
        sockO = (socket.socket(socket.AF_INET,      # IP
                               socket.SOCK_DGRAM))  # UDP
        while True:
            try:
                sockO.sendto(pipe_in.recv(), (ip_out, port_out))  # here it is being supposed the pipe has the final string
                                                                  # the method .recv() is a blocking command

            except IOError: # Without the IOError even the keyboard "control+C" is caught here!
                print "Send_UDP IO error?"

            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # Kills the while...

    @staticmethod
    def publish_to_topic(topic_name, pipe_in, msg_type, max_freq=10):
        """Very simple function to publish data received from a multiprocessing.Pipe to a ROS Topic.
        Args:
            node_name (str): an unique name for your ROS node.
            topic_name (str): ROS topic to publish.
            pipe_in (multiprocessing.Pipe): Pipe where the data is coming from.
            msg_type (str): name of the ROS std_msgs.msg type used.
            max_freq (int): limits the maximum frequency data is published.
        """

        pub = rospy.Publisher(topic_name, getattr(std_msgs.msg, msg_type), tcp_nodelay=True, queue_size=1)

        rospy.init_node("UDP_receiver", anonymous=True)

        rate = rospy.Rate(max_freq) # 10hz

        while not rospy.is_shutdown():
            data = pipe_in.recv()
            pub.publish(data)
            rospy.loginfo("Data received from UDP: " + data)
            rate.sleep()

    @staticmethod
    def receive_from_topic(topic_name, pipe_out, msg_type):
        """Very simple function to receive data from a ROS Topic and pass to a multiprocessing.Pipe.
        Args:
            node_name (str): an unique name for your ROS node.
            topic_name (str): ROS topic to read from.
            pipe_out (multiprocessing.Pipe): Pipe where the data will be sent.
            msg_type (str): name of the ROS std_msgs.msg type used.
        """

        rospy.init_node("UDP_sender", anonymous=True)

        def callback(data):
            pipe_out.send(data)
            rospy.loginfo("Data sent to UDP: " + data)

        rospy.Subscriber(topic_name, getattr(std_msgs.msg, msg_type), callback)

        rospy.spin()

if __name__=="__main__":


    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the UDP2ROS bridge.")

            ip_in (str): IP adress to receive UDP packets.
            port_in (int): port number to receive UDP packets.
            ip_out (str): IP adress to send UDP packets.
            port_out (int): port number to send UDP packets.
            topic_to_publish_to (str): topic name
            topic_to_subscribe_to (str): topic name
            msg_type (str): ROS std_msgs.msg
            max_freq (int): max frequency to publish.
            buffer_size (int): socket buffer size (bytes).
            clean_loop (bool): clean the buffer before start.

    parser.add_argument("--ip_in", help="IP adress to receive UDP packets.", type=str, required=True)
    parser.add_argument("--port_in", help="port number to receive UDP packets.", type=int, required=True)
    parser.add_argument("--ip_out", help="IP adress to send UDP packets.", type=str, required=True)
    parser.add_argument("--port_out", help="port number to send UDP packets.", type=int, required=True)
    parser.add_argument("--topic_to_publish_to", help="ROS topic name to publish received UDP packets.", type=str, default="send_UDP")
    parser.add_argument("--topic_to_subscribe_to", help="ROS topic name to subscribe to send UDP packets.", type=str, default="receive_UDP")
    parser.add_argument("--msg_type", help="ROS std_msgs.msg", type=str, default="String")
    parser.add_argument("--max_freq", help="max frequency to publish", type=int, default=10)
    parser.add_argument("--buffer_size", help="socket receiver buffer size (bytes).", type=int, default=1024)
    parser.add_argument("--clean_loop", help="clean the buffer before start.", type=int, default=1)

    args=parser.parse_args()

    new_bridge =  UDP2ROS(args.ip_in, args.port_in, args.ip_out, args.port_out, args.topic_to_publish_to, args.topic_to_subscribe_to,
                          args.msg_type, args.max_freq, args.buffer_size, args.clean_loop)
    new_bridge.start()
