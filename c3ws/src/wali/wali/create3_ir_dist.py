#!/bin/env python3

# FILE:  create3_ir_dist.py

# Create3 ir_intensity values to *approximate* distance
#   using linear interpolation between empirical test values

#   The ir sensors are not albedo (surface reflectivity) independent
#   so the distances should not be granted great confidence

import numpy as np


LABELS = ["side_left", "left", "front_left", "front_center_left", \
              "front_center_right", "front_right", "right"]

# Approximate distance within range of 1cm to 60cm (less or more undefined)
DISTANCES = [0.520, 0.420, 0.320, 0.220, 0.120, 0.070, 0.045, 0.035]

# Average Reading Values For Distances
READINGS  = [
             [ 9    , 14   , 19   ,  73   , 336  , 1148  , 2469 , 3111],
             [17    , 21   , 35   ,  74   , 322  , 1252  , 3090 , 3481],
             [31    , 44   , 53   , 112   , 366  , 1162  , 2639 , 3412],
             [13    , 23   , 42   , 126   , 451  , 1385  , 2843 , 3514],
             [13    , 23   , 42   , 126   , 451  , 1385  , 2843 , 3514],
             [13    , 23   , 42   , 126   , 451  , 1385  , 2843 , 3514],
             [13    , 23   , 42   , 126   , 451  , 1385  , 2843 , 3514],
            ]

def dist_ir_reading(sensor_idx, reading):
      UNDEF = -99.999
      dist = np.interp(reading,READINGS[sensor_idx],DISTANCES, right=UNDEF, left=UNDEF)
      return dist

def main():

  for reading in [0, 8, 10, 13, 25, 38, 57, 76, 114, 151, 366, 580, 1312, 2044, 2908, 3772, 3785, 3798, 4500] :
    print("reading: {}  dist: {:.3f} m".format(reading, dist_ir_reading(4,reading)))


if __name__ == "__main__":
  main()
