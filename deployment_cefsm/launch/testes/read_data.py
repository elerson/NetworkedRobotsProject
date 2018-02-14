#!/usr/bin/python
import math
from datetime import datetime
import csv
format = '%H:%M:%S.%f'

def main():
    resolution = 0.025
    initial_position = (0.0, 0.0)
    file = open('output1.txt', 'r')
    position_str = file.read().split('\n')

    file = open('rss_teste1.txt', 'r')
    rss_str = file.read().split('\n')

    n_positions = len(position_str)
    n_rss = len(rss_str)

    r = 0
    p = 0
    rss_value = '-1'
    data = []
    with open('data.csv', 'wb') as csvfile:
        wr = csv.writer(csvfile)
        while(True):
            rss_ = rss_str[r].split(' ')
            position_ = position_str[p].split(';')

            rss_time = datetime.strptime(rss_[0], format)
            p_time = datetime.strptime(position_[0], format)

            if rss_time < p_time:

                rss_value = rss_[6]

                r = r + 1
            else:

                pos_ = eval(position_[1])
                if(initial_position[0] == 0.0 and initial_position[1] == 0.0 and pos_[0] != 0 and pos_[1] != 0):
                    initial_position = (pos_[0]*resolution, pos_[1]*resolution)
                elif(pos_[0] == 0 and pos_[1] == 0):
                    p = p + 1
                    continue


                pos = math.sqrt((initial_position[0] - pos_[0]*resolution)**2 + (initial_position[1] - pos_[1]*resolution)**2)
                var = eval(position_[2])
                wr.writerow((pos, var[0], var[1] , float(rss_value[0:3])))
                

                p = p + 1

            if(p >= n_positions-1 or r >= n_rss-1 ):
                break
    
        
        



if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function