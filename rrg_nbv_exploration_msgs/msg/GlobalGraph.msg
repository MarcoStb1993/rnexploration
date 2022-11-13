# Describes a global graph constructed with RRG
std_msgs/Header header
string ns

GlobalTarget[] targets		  					# Array of all targets in this global graph with the first entry being the origin
GlobalConnection[] connections						# Array of all connections between targets and from targets to the local graph in this global graph
int32 targets_counter				   			# Number of targets in the targets array
int32 connections_counter   					# Number of connections in the connections array