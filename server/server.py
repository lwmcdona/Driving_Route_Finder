'''
1st Partner: Logan McDonald (TR Section)
2nd Partner: Veronica Salm (WF Section)
CMPUT 275 LBL EB1/LBL B2
Winter 2017

Code Filename: server.py

COLLABORATED CODE: This assignment was developed collaboratively by both partners.

ACCESSORIES: Before running the server protocol from the command line, an Arduino
    must be connected and be running the client portion of the code. See the included
    README for detailed instructions.

GENERAL NOTES / PROBLEM AREAS:
    The finite state machine used to implement the server protocol will continue
    to run indefinitely.

    In the case of bad input at any stage (incorrectly formatted requests or
    improper acknowledgement), the server returns to the WAIT state and waits for
    more input. In the case of input that will not convert properly to integers,
    a try-except block is used to handle the error and the server again remains in
    the WAIT state.

    Also note that the least_cost_path() function has a long run-time when there
    is no available path between vertices. This could be improved by using
    breadth-first-search to determine if the destination vertex is connected; but
    as BFS would increase run-time in most cases, we did not implement it.
'''
import csv
import math
from adjacencygraph import AdjacencyGraph as Graph
import heapq
from cs_message import *
import time

import profile # testing librairies - used to test run-time
import pstats

from enum import Enum #for FSM
import sys

def least_cost_path(graph, start, dest, cost):
    '''Find and return a least cost path in graph from start vertex to dest vertex.

    Efficiency: If E is the number of edges, the run-time is O( E log(E) ).

    Args:
        graph (Graph): The digraph defining the edges between the vertices.
        start: The vertex where the path starts. It is assumed that start is a
            vertex of graph.
        dest:  The vertex where the path ends. It is assumed that start is a
            vertex of graph.
        cost:  A function, taking the two vertices of an edge as parameters and
            returning the cost of the edge. For its interface, see the definition
            of cost_distance.

    Returns:
        list: A potentially empty list (if no path can be found) of
            the vertices in the graph. If there was a path, the first
            vertex is always start, the last is always dest in the list.
            Any two consecutive vertices correspond to some
            edge in graph.
    '''
    #For this function, a runner is described as a set of three values: (t_arrive, v_from, v_to)
    # t_arrive is the time of arrival at a specified vertex
    # v_from is the vertex from which the runner originated
    # v_to is the specified vertex

    # To keep track of when the first runner reached a vertex and from which vertex
    # denoted by v_from
    reached = dict()

    # Initially, we push one runner onto the heap,
    # running from the start vertex to the start vertex
    runners = [] # a list of all runners (treated as a heap)
    heapq.heappush(runners,(0, (0, start, start))) # push the first runner onto the heap

    found = False #indicating that so far no path has been found
    # while there are still places to go
    while len(runners) > 0:
        (key, first_runner) = heapq.heappop(runners) # heappop() removes the smallest item from the heap
        # key is used in the heap implementation to ensure heap properties

        (t_arrive, v_from, v_to) = first_runner

        if v_to in reached: #if the vertex has already been reached (arrived at time <= t_arrive),
        # discard the runner and continue to the next iteration
            continue

        # first to arrive at v_to came from v_from
        reached[v_to] = (t_arrive, v_from)

        if v_to == dest: # if a runner has reached the destination, a path has been found
            found = True
            break # no need to check any further

        # otherwise, spawn more runners to each neigbour of v_to
        # and add them to the heap
        for v_next in graph.neighbours(v_to):
            # Add a new runner event for each outgoing edge v_to -> v_next
            # provided that v_next has not been reached already.
            if v_next in reached:
                continue

            # time to arrive at the next edge is:
            t = t_arrive + cost(v_to, v_next)
            heapq.heappush(runners,(t, (t, v_to, v_next))) # push the runner onto the heap

    shortpath = []
    if found: # work backwards from dest to start, appending each vertex to the path
        i = dest
        shortpath.append(i)
        while i != start:
            i = reached[i][-1]
            shortpath.append(i)
        shortpath.reverse() # then reverse the list to print it in the correct order

    return shortpath

def read_city_graph(filename):
    '''Creates the graph of a CSV (Comma-Separated Values) file
    describing a directed road network.

    Lines of the file can be formatted in two ways:
    1. V,ID,Lat,Lon: Describes a vertex in the graph
        V (character): the character 'V' denoting vertex
        ID (integer): the name or identifier of the vertex
        Lat, Lon (two floating point numbers): correspond to the latitude and
            longitude of a vertex's location on the map.
    2. E,start,end,name: Describes an edge in a graph
        E (character): the character 'E' denoting edge
        start, end (two integers): two vertices corresponding to the beginning
            and end of an edge
        name (string): name of the street on the map

    Args:
        filename (string) : the name of the file to be read, in the form "filename.txt",
            where the file must be in CSV format.

    Returns:
        graph: An instance of a directed graph, including all the vertices and edges read
        from the file.
        cost_distance (function): The nested cost_distance function. For its interface, see the definition
            of cost_distance.
        min_vertex (function): The nested min_vertex function.For its interface, see the definition
            of min_vertex.
        coords (dict): A set of the coordinates of each vertex on the graph in the format
            {vertex ID: (lat, lon)}.
    '''
    g = Graph()
    coords = dict() # a set of (lat, lon) tuples
    # streetnames = dict() # Not needed for this part but will be needed later
    with open(filename) as f:
        reader = csv.reader(f)
        count = 1 # used to specify the row if a formatting error is found in the CSV file

        for row in reader:
            if row[0] == 'V':
                g.add_vertex(int(row[1]))
                lat = int(float(row[2])*100000)
                lon = int(float(row[3])*100000)
                coords[int(row[1])] = (lat, lon)

            elif row[0] == 'E':
                t = (int(row[1]), int(row[2]))
                g.add_edge(t) #autocreation = True
                # streetnames[int(row[1]), int(row[2])] = row[3]
            else:
                raise RuntimeError("File improperly formatted. Row: {}".format(count))
            count += 1

        def cost_distance(u, v):
            '''Computes and returns the straight-line distance between the two vertices u and v.

            Args:
                u, v: The IDs for two vertices that are the start and end of a valid
                    edge in the graph.

            Returns:
                numeric value: the distance between the two vertices.
            '''
            num = math.sqrt(  ((coords[u][0]-coords[v][0])** 2)
                            + ((coords[u][1]-coords[v][1])** 2))
            return num

        def min_vertex(point):
            '''Computes and returns the nearest vertex to a given point on the
                graph.

            Args:
                point (a tuple of two ints): A tuple in the form (lat, lon) that
                    contains the latitude and longitude of the given point.

            Returns:
                vertex: The identifier of the nearest vertex.
            '''
            min_distance = float("inf") # sets the starting distance to infinity
            for v in g.vertices():
                # computes the distance between a point and each vertex and stores the minimum
                distance = math.sqrt( ((point[0] - coords[v][0])**2) + ((point[1] - coords[v][1])**2))
                if distance < min_distance:
                    min_distance = distance
                    vertex = v
            return vertex

    return (g, cost_distance, min_vertex, coords)

def protocol(serial_in, serial_out):
    ''' The server protocol for communicating with the Arduino client. Contains
        a finite state machine (FSM) enumerating the server's states. The server
        receives, computes, and returns the path between two waypoints to the
        client indefinitely.

    Args:
        serial_in, serial_out: The serial ports used for communication with the client,
        imported from textserial.py.

    Returns:
        void: Nothing is returned as the while loop is designed to run indefinitely.
    '''
    (graph, cost_distance, min_vertex, coords) = read_city_graph("edmonton-roads-2.0.1.txt")

    class FSM (Enum):
        '''The finite state machine enumerating three possible states for the server.'''
        WAIT = 0 # wait for input
        ACK = 2 # wait for acknowledgement
        SEND = 3 # send information: send vertices in order, send E when done

    state = FSM.WAIT

    while True:
        msg = receive_msg_from_client(serial_in) # get a message from the client
        log_msg(msg)

        if state == FSM.WAIT:
            # log_msg("wait")
            line = msg.split()

            if len(line) == 5: #this checks that the line is not empty and that the length is correct
            # recall that input should be in the form "R int int int int<\n>", so length should be 5
                if line[0] == "R":
                    # extract the start and end points (point1 and point2)
                    try:
                        (int(line[1]), int(line[2]))
                        (int(line[3]), int(line[4]))
                    except Exception:
                        continue # if the numbers are not integers, stay in the wait state and get more input
                    else:
                        point1 = (int(line[1]), int(line[2]))
                        point2 = (int(line[3]), int(line[4]))
                    # and find the closest vertices on the graph
                    start = min_vertex(point1)
                    dest = min_vertex(point2)

                    # find the least cost path between start and dest
                    path = least_cost_path(graph, start, dest, cost_distance)
                    count = 0 # to keep track of the number of vertices that have been sent
                    send_msg_to_client(serial_out, "N {}" .format(len(path)))
                    if len(path) > 0:
                        state = FSM.ACK # change state to acknowledge
            # note that in the case of bad input or a path of length 0, the FSM stays in
            # state 1 and waits for correct input

        elif state == FSM.ACK:
            # log_msg("ack")
            msg = msg.strip()

            print(msg == "A")
            if msg == "A": # after every acknowledgement, move to the SEND state
                state = FSM.SEND
            else: #for bad input
                log_msg("acknowledgement failed")
                state = FSM.WAIT #start over
        if state == FSM.SEND:
            # log_msg("send")
            if count < len(path): #if there are still vertices left
                send_msg_to_client(serial_out, "W {} {}" .format(coords[path[count]][0], coords[path[count]][1]))
                count += 1
                state = FSM.ACK
            else:
                send_msg_to_client(serial_out, "E")
                state = FSM.WAIT #wait for more input

def main():
    ''' Initializes important arguments using argparse and runs the
    server protocol. The code in this function is from the dummy_server.py
    file found on eclass.
    '''
    import argparse
    parser = argparse.ArgumentParser(
        description='Client-server message test.',
        formatter_class=argparse.RawTextHelpFormatter,
        )

    parser.add_argument("-d0",
        help="Debug off",
        action="store_false",
        dest="debug")

    parser.add_argument("-s",
        help="Set serial port for protocol",
        nargs="?",
        type=str,
        dest="serial_port_name",
        default="/dev/ttyACM0")

    args = parser.parse_args()

    debug = args.debug

    set_logging(debug)

    # this imports serial, and provides a useful wrapper around it
    import textserial

    serial_port_name = args.serial_port_name;
    log_msg("Opening serial port: {}".format(serial_port_name))

    # Open up the connection
    baudrate = 9600  # [bit/seconds] 115200 also works

    # The with statment ensures that if things go bad, then ser
    # will still be closed properly.
    # errors='ignore' allows any 1 byte character, not just the usual
    # ascii range [0,127]
    with textserial.TextSerial(
        serial_port_name, baudrate, errors='ignore', newline=None) as ser:
        protocol(ser, ser)

if __name__ == "__main__": # the code within this if statement will only run if the program is run from the command line
    main()
