#!/usr/bin/python

from tree import *
import sys

def main():

    filename = "/home/elerson/catkin_ws/src/networked_robots_experimetal_data/1euclideanexperiment/steinerData1.dat"
    tree = Tree(filename)
    tree_segmentation = TreeSegmention(tree)

    closed_list = []
    paths = []
    tree_segmentation.segmentation_search(closed_list, paths)
    #print(tree_segmentation.segmentaion_paths)
    segmentation = tree_segmentation.evaluate_segmentation()
    print(segmentation)

if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function