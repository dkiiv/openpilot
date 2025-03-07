import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.stats import describe
from math import erf, fabs, atan, cos, exp

from selfdrive.config import Conversions as CV

from tools.tuning.lat_settings import *

PI = 3.14159
  
def clip(v,l,h):
  return min(h, max(v, l))

def get_sigmoid_coef(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT):
  return np.vectorize(SIGMOID_COEF_RIGHT if angle > 0. else SIGMOID_COEF_LEFT)


def get_sigmoid_coef_torque(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT):
  return np.vectorize(SIGMOID_COEF_RIGHT if angle < 0. else SIGMOID_COEF_LEFT)

def get_slope_torque(angle, B, C):
# B is right
  return np.vectorize(B if angle < 0. else C)

def get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, C, SIGMOID_SPEED, SIGMOID, SPEED):
  x = ANGLE * (desired_angle + C)
  sigmoid = x / (1 + fabs(x))
  return (SIGMOID_SPEED * sigmoid * v_ego) + (SIGMOID * sigmoid) + (SPEED * v_ego)
 

def get_steer_feedforward_sigmoid1(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (angle) / max(0.01,speed)
  sigmoid = x / (1. + fabs(x))
  return ((SIGMOID_COEF_RIGHT if (angle) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + speed + D) ** B + C * ((angle) * SPEED_COEF - atan((angle) * SPEED_COEF))

def get_steer_feedforward_erf_old(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (angle + C) * (40.23 / (max(1.0,speed + D))**SPEED_COEF)
  sigmoid = erf(x)
  sigmoid *= (SIGMOID_COEF_RIGHT if (angle + C) < 0. else SIGMOID_COEF_LEFT)
  linear = B * (angle + C)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (angle + C) * (40.23 / (max(1.0,speed + D))**SPEED_COEF)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= (SIGMOID_COEF_RIGHT if (angle + C) < 0. else SIGMOID_COEF_LEFT)
  linear = B * (angle + C)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid1(lateral_accel_value, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = A * (lateral_accel_value + C) * (40.23 / (max(1.0,v_ego + D))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (lateral_accel_value + C) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= max(0.2, 40.23 / (max(1.0,v_ego + SPEED_OFFSET2))**SPEED_COEF2)
  linear = B * (lateral_accel_value + C)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid_to_linear(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = A * (angle + C) * (40.23 / (max(1.0,speed + D))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + C) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= sigmoid_factor
  linear = B * (angle + C)
  sigmoid += linear
  
  linear_solo = angle * SPEED_COEF2
  max_speed = 30.
  speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
  
  out = (1-speed_norm) * linear_solo + (speed_norm) * sigmoid
  
  return out

def get_steer_feedforward_erf1(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = A * (angle + C) * (40.23 / (max(0.05,speed + D))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + C) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = erf(x)
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= (40.23 / (max(0.05,speed + SPEED_OFFSET2))**SPEED_COEF2)
  linear = B * (angle + C)
  return sigmoid + linear


def get_steer_feedforward_bolt_euv_old(desired_angle, v_ego):
  ANGLE = 0.0758345580739845
  C = 0.#31396926577596984
  SIGMOID_SPEED = 0.04367532050459129
  SIGMOID = 0.43144116109994846
  SPEED = -0.002654134623368279
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, C, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_bolt_euv(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = A * (angle + C)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B)))

def get_steer_feedforward_bolt_euv_torque(desired_lateral_accel, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (desired_lateral_accel + C) * (40.23 / (max(1.0,speed + D))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + C) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + B * (desired_lateral_accel + C)


def get_steer_feedforward_lacrosse(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = A * (angle + C)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B)))

def get_steer_feedforward_lacrosse_torque(desired_lateral_accel, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (desired_lateral_accel + C) * (40.23 / (max(0.05,speed + D))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + C) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + B * (desired_lateral_accel + C)
  

def get_steer_feedforward_suburban_old(desired_angle, v_ego):
  ANGLE = 0.06562376600261893
  C = 0.#-2.656819831714162
  SIGMOID_SPEED = 0.04648878299738527
  SIGMOID = 0.21826990273744493
  SPEED = -0.001355528078762762
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, C, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_suburban(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = A * (angle + C)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B)))

def get_steer_feedforward_suburban_torque_old(angle, speed):
    A = 0.66897758
    B = 0.01000000
    C = -0.44029828
    D = 2.31755298
    SIGMOID_COEF_RIGHT = 0.35709901
    SIGMOID_COEF_LEFT = 0.36136769
    SPEED_COEF = 0.13870482
    
    x = A * (angle + C)
    sigmoid = x / (1. + fabs(x))
    
    linear = angle
    max_speed = 26.
    speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
    
    return (speed_norm) * linear + (1-speed_norm) * (((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B))))
  
def get_steer_feedforward_palisade_torque_old(angle, speed):
    linear = angle
    linear2 = angle * 0.39
    max_speed = 26.
    speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
    
    return (speed_norm) * linear + (1-speed_norm) * linear2


def get_steer_feedforward_suburban_torque(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = A * (angle + C)
    sigmoid = x / (1. + np.fabs(x))
    
    return ((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B)))

def get_steer_feedforward_bolt_old(desired_angle, v_ego):
  ANGLE = 0.06370624896135679
  C = 0.#32536345911579184
  SIGMOID_SPEED = 0.06479105208670367
  SIGMOID = 0.34485246691603205
  SPEED = -0.0010645479469461995
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, C, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_bolt_torque_old(desired_lateral_accel, speed):
  A = 0.79289935
  B = 0.24485508
  D = 1.00000000
  SIGMOID_COEF_RIGHT = 0.30436939
  SIGMOID_COEF_LEFT = 0.22542412
  SPEED_COEF = 0.77320476
  C=0.0
  x = A * (desired_lateral_accel + C) * (40.23 / (max(0.05,speed + D))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + C) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + B * (desired_lateral_accel + C)
  
def get_steer_feedforward_bolt(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = A * (angle + C)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + C) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + D) * SPEED_COEF) * ((fabs(angle + C) ** fabs(B)))

def get_steer_feedforward_bolt_torque(desired_lateral_accel, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = A * (desired_lateral_accel + C) * (40.23 / (max(0.05,speed + D))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + C) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + B * (desired_lateral_accel + C)

def get_steer_feedforward_volt_old(desired_angle, v_ego):
  A = 1.23514093
  B = 2.00000000
  C = 0.03891270
  D = 8.58272983
  SIGMOID_COEF_RIGHT = 0.00154548
  SIGMOID_COEF_LEFT = 0.00168327
  SPEED_COEF = 0.16283995
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
# def get_steer_feedforward_volt_torque_old(desired_lateral_accel, v_ego):
#   A = 0.08617848
#   B = 0.14
#   C = 0.00205026
#   D = -3.48009247
#   SIGMOID_COEF_RIGHT = 0.56664089
#   SIGMOID_COEF_LEFT = 0.50360594
#   SPEED_COEF = 0.55322718
#   return get_steer_feedforward_erf_old(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_volt_torque_old(desired_lateral_accel, v_ego):#, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  A = 0.15461558
  B = 0.22491234
  C = -0.01173257
  D = 2.37065180
  SIGMOID_COEF_RIGHT = 0.14917052
  SIGMOID_COEF_LEFT = 0.13559770
  SPEED_COEF = 0.49912791
  SPEED_COEF2 = 0.37766423
  SPEED_OFFSET2 = -0.36618369
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

def get_steer_feedforward_volt(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  # A = 1.23514093
  # B = 2.00000000
  # C = 0.03891270
  # D = 8.58272983
  # SIGMOID_COEF_RIGHT = 0.00154548
  # SIGMOID_COEF_LEFT = 0.00168327
  # SPEED_COEF = 0.16283995
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_volt_torque(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  # A = 0.09096546
  # B = 0.12402084
  # C = 0.
  # D = -3.35899817
  # SIGMOID_COEF_RIGHT = 0.48819415
  # SIGMOID_COEF_LEFT = 0.55110842
  # SPEED_COEF = 0.57397696
  # return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
  # return get_steer_feedforward_torque_sigmoid1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
  return get_steer_feedforward_torque_sigmoid1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_acadia_torque(desired_lateral_accel, v_ego):#, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  A = 0.32675089
  B = 0.22085755
  C = 0.
  D = -3.17614605
  SIGMOID_COEF_RIGHT = 0.42425039
  SIGMOID_COEF_LEFT = 0.44546354
  SPEED_COEF = 0.78390078
  return get_steer_feedforward_erf_old(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_escalade(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_escalade_torque(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_silverado(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_silverado_torque(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# For comparison with previous best
def old_feedforward(speed, angle):
  
  # sierra silverado combined
  # A = 0.06539361463056717
  # D = -0.8390269362439537
  # SIGMOID_COEF_LEFT = 0.023681877712247515
  # SIGMOID_COEF_RIGHT = 0.5709779025308087
  # SPEED_COEF = -0.0016656455765509301
  
  #sierra only
  # A = 0.07375408334531243
  # D = -0.43842460609320844
  # SIGMOID_COEF_LEFT = 0.015039986300916987
  # SIGMOID_COEF_RIGHT = 0.6154522080649616
  # SPEED_COEF = -0.00238195057681674
  
  # silverado only
  # A = 0.07017408594582242
  # D = -0.7108582322213549
  # SIGMOID_COEF_LEFT = 0.02534582973830592
  # SIGMOID_COEF_RIGHT = 0.5901819029949994
  # SPEED_COEF = -0.0026961086215487357
  # return feedforward(speed, angle, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  # return 0.0002 * (speed ** 2) * angle # old bolt and bolteuv
  # return 0.00004 * (speed ** 2) * angle # old silverado/sierra
  # return 0.000195 * (speed ** 2) * angle # old suburban


  
  if IS_ANGLE_PLOT:
    # old volt sigmoid
    # x = angle * 0.02904609
    # sigmoid = x / (1 + np.fabs(x))
    # return 0.10006696 * sigmoid * (speed + 3.12485927)
    #old acadia sigmoid
    # desired_angle = 0.09760208 * angle
    # sigmoid = desired_angle / (1 + np.fabs(desired_angle))
    # return 0.04689655 * sigmoid * (speed + 10.028217)
    # current acadia sigmoid
    A = 5.00000000
    B = 1.90844451
    ANGLE_COEF3 = 0.03401073
    D = 13.72019138
    SIGMOID_COEF_RIGHT = 0.00100000
    SIGMOID_COEF_LEFT = 0.00101873
    SPEED_COEF = 0.36844505
    return np.vectorize(get_steer_feedforward_sigmoid1)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

    # return np.vectorize(get_steer_feedforward_suburban_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_old)(angle, speed)
    # return 0.000195 * (speed ** 2) * angle # old suburban
    return 0.0002 * (speed ** 2) * angle # old bolt and bolteuv
    return 0.00004 * (speed ** 2) * angle # old silverado/sierra, gm default
    # return 0.000045 * (speed ** 2) * angle # old escalade
    # return 0.00005 * (speed ** 2) * angle
  else:
    # return np.vectorize(get_steer_feedforward_bolt_torque_old)(angle, speed)
    # kf = .33 # old volt torque, lacrosse
    # kf = 0.4 # old tahoe torque
    # kf = 0.35 # sonata 
    # return np.vectorize(get_steer_feedforward_palisade_torque_old)(angle,speed)
    # kf = 0.393 # palisade 
    # kf = 1/1.4 # vw (golf, passat)
    kf = 1/2.9638737459977467 # sonata 2020
    # kf = 1/2.544642494803999 # palisade 2020
    # kf = 1/2 # ram 1500
    # kf = 1/1.593387270257916 #pacifica 2018
    # return np.vectorize(get_steer_feedforward_volt_torque_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_acadia_torque)(angle, speed)
    return angle * kf
    
    # return np.vectorize(get_steer_feedforward_suburban_torque_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_torque_old)(angle, speed)
    
    # current acadia
    # A = 0.32675089
    # B = 0.22085755
    # C = 0.
    # D = -3.17614605
    # SIGMOID_COEF_RIGHT = 0.42425039
    # SIGMOID_COEF_LEFT = 0.44546354
    # SPEED_COEF = 0.78390078
    # return np.vectorize(get_steer_feedforward_erf1)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  



def new_feedforward(speed, angle):
  return np.vectorize(torque_from_lateral_accel_siglin)(speed, angle, A, B, C, D, E, F, G, H)

def feedforward(speed, angle, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  
  return np.vectorize(torque_from_lateral_accel_siglin)(speed, angle, A, B, C, D, E, F, G, H)
  if IS_ANGLE_PLOT:
    # return A * angle * speed ** SPEED_COEF # tahoe angle fit

    
    # great for volt
    return np.vectorize(get_steer_feedforward_sigmoid1)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # x = A * (angle) / np.fmax(0.01,speed)
    # sigmoid = x / (1. + np.fabs(x))
    # return (np.vectorize(get_sigmoid_coef)(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + speed + D) ** B + C * (angle * SPEED_COEF - np.arctan(angle * SPEED_COEF))
    
    # return np.vectorize(get_steer_feedforward_bolt_euv)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    
    return np.vectorize(get_steer_feedforward_lacrosse)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    return np.vectorize(get_steer_feedforward_suburban)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  else:
    # return A * (angle + C) / np.log(np.fmax(1.0, speed)) / (np.log(SPEED_COEF)) 
    
    # bolt
    # x = A * (angle + C) * (40.23 / (np.fmax(0.05,speed + D))**SPEED_COEF)
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle + C) * sigmoid) + B * (angle + C)
    
    # return np.vectorize(get_steer_feedforward_bolt_euv_torque)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # return np.vectorize(get_steer_feedforward_lacrosse_torque)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    
    # suburban
    return np.vectorize(get_steer_feedforward_volt_torque)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
    
    # great for volt
    # return np.vectorize(get_steer_feedforward_erf1)(angle, speed, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # x = A * (angle) * (40.23 / (np.fmax(0.05,speed + D))**SPEED_COEF)
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT) * sigmoid) + B * angle
    
    # great for silverado/sierra
    # x = A * (angle) * (40.23 / (np.fmax(0.2,speed)))
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle) * sigmoid) + np.vectorize(get_slope)(angle) * angle
    
    # x = A * (angle)
    # # sigmoid = x / (1. + np.fabs(x))
    # sigmoid = np.vectorize(erf)(x)
    # # sigmoid = np.arcsinh(x)
    # return (np.vectorize(get_sigmoid_coef)(angle) * sigmoid) + 0.1 * angle

# def feedforward(speed, angle, A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):

#   return angle * A / (np.maximum(speed - D, 0.1) * SPEED_COEF)**SIGMOID_COEF_LEFT

def torque_from_lateral_accel_siglin(speed, lataccel, a, b, c, d, e, f, g, h):

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    speed_factor = (40.23 / (max(1.0,speed + e))**f)
    speed_factor2 = max(0.2, 40.23 / (max(1.0,speed + g))**h)
    steer_torque = (sig(lataccel * a * speed_factor) * b * speed_factor2) + (lataccel * c) + d
    return float(steer_torque)

def _fit_kf(x_input, A, B, C, D, E, F, G, H):
  speed, angle = x_input.copy()
  return np.vectorize(torque_from_lateral_accel_siglin)(speed, angle, A, B, C, D, E, F, G, H)

def fit(speed, angle, steer, angle_plot=True):
  global IS_ANGLE_PLOT
  IS_ANGLE_PLOT = angle_plot
  
  print(f'speed: {describe(speed)}')
  print(f'angle: {describe(angle)}')
  print(f'steer: {describe(steer)}')
  
  print("Performing fit...")
  
  global A, B, C, D, E, F, G, H
  BOUNDS = ([0.001, .3, 0.25, -1.0, 15.0, 0.1, 30.0, 0.1],
            [20.0, 2.0, 1.0, 1.0, 40.0, 2.0, 40.0, 1.0])
  params, _ = curve_fit(  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
    _fit_kf,
    np.array([speed, angle]),
    np.array(steer),
    maxfev=900000,
    bounds=BOUNDS
  )
  A, B, C, D, E, F, G, H = params
  print(f'Fit: {params}')
  i = 0
  print(f"{A = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{B = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{C = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{D = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{E = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{F = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{G = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{H = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  print(f"{BOUNDS = }")

  new_residual = np.fabs(new_feedforward(speed, angle) - steer)
  new_mae = np.mean(new_residual)
  print('MAE new {}'.format(round(new_mae, 4)))
  new_std = np.std(new_residual)
  print('STD new {}'.format(round(new_std, 4)))
  
  if not os.path.exists("plots"):
    os.mkdir("plots")
  with open("plots/out.txt","a") as f:
    f.write(f"    {A = :.8f}\n")
    f.write(f"    {B = :.8f}\n")
    f.write(f"    {C = :.8f}\n")
    f.write(f"    {D = :.8f}\n")
    f.write(f"    {E = :.8f}\n")
    f.write(f"    {F = :.8f}\n")
    f.write(f"    {G = :.8f}\n")
    f.write(f"    {H = :.8f}\n")
    f.write('mean absolute error: new {}\n'.format(round(new_mae, 4)))
    f.write('standard deviation: new {}\n'.format(round(new_std, 4)))
    f.write(f"fit computed using {len(speed)} points")

def plot(speed, angle, steer):
  if SPEED_PLOTS:
    os.system('rm plots/deg*')
    abs_angle = np.fabs(angle)
    abs_steer = np.fabs(steer)

    # if PLOT_ANGLE_DIST:
    #   sns.distplot([
    #       line['angle'] for line in data if abs(line['angle']) < 30
    #   ],
    #                bins=200)
    #   raise Exception

    res = 1000

    if IS_ANGLE_PLOT:
      _angles = []
      STEP = 3 # degrees
      for a in range(0, 45, STEP):
        _angles.append([a, a + STEP])
      _angles = np.r_[_angles]
    else: # lateral acceleration plot
      _angles = []
      STEP = 0.2 # degrees
      astart = 0.
      aend = 5.
      for a in np.linspace(astart, aend, num=int((aend-astart)/STEP)).tolist():
        _angles.append([a, a + STEP])
      _angles = np.r_[_angles]

    angle_points_dict = {}
    
    for angle_range in _angles:
      if IS_ANGLE_PLOT:
        start = round(angle_range[0])
        end = round(angle_range[1])
        angle_range_str = f'deg {start:02d}-{end:02d}'
      else:
        start = angle_range[0]
        end = angle_range[1]
        angle_range_str = f'lat_accel {start:.2f}-{end:.2f}'
      mask = (angle_range[0] <= abs_angle) & (abs_angle <= angle_range[1])

      plot_speed = speed[mask]
      plot_angle = abs_angle[mask]
      plot_steer = abs_steer[mask]

      params = None
      if FIT_EACH_PLOT and sum(mask) > 4:
        try:
          global A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF
          params, _ = curve_fit(  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
            _fit_kf,
            np.array([plot_speed, plot_angle]),
            np.array(plot_steer),
            maxfev=9000,
          )
          A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF = params
        except RuntimeError as e:
          print(e)
          continue
      
      angle_points_dict[angle_range_str] = len(plot_speed)
      # print(f'{angle_range_str} ({len(plot_speed)}): {params}')
      plt.figure(figsize=(12,8))
      plt.scatter(plot_speed * CV.MS_TO_MPH,
                  plot_steer,
                  label=angle_range_str,
                  color='black',
                  s=1.)

      _x_ff = np.linspace(0, 80, res)
      _y_ff = [
          old_feedforward(_i, np.mean(angle_range))
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='red',
               label='old')

      _y_ff = [
          new_feedforward(_i, np.mean(angle_range))
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='blue',
               label='new')

      plt.title(angle_range_str)
      plt.legend(loc='upper left')
      plt.xlabel('speed (mph)')
      plt.ylabel('steer')
      plt.ylim(0., 1.5)
      plt.xlim(SPEED_MIN, SPEED_MAX)
      plt.grid(axis='x', color='0.95')
      plt.grid(axis='y', color='0.95')
      if not os.path.isdir('plots'):
        os.mkdir('plots')
      plt.savefig(f'plots/{angle_range_str}.png')
      plt.close()
  
    print(''.join([ "{}:{}{}".format(k,v,'\n' if (ki+1) % 5 == 0 else ', ') for ki,(k,v) in enumerate(sorted(list(angle_points_dict.items()), key=lambda x: x[0]))]))
    

  if ANGLE_PLOTS:
    os.system('rm plots/mph*')
    # if PLOT_ANGLE_DIST:
    #   sns.displot([
    #       line['angle'] for line in data if abs(line['angle']) < 30
    #   ],
    #               bins=200)
    #   raise Exception

    res = 1000

    _speeds = []
    STEP = 10 # mph
    for s in range(SPEED_MIN, 90, STEP):
      _speeds.append([s, s + STEP])
    _speeds = np.r_[_speeds]

    angle_points_dict = {}
    for speed_range in _speeds:
      start = round(speed_range[0])
      end = round(speed_range[1])
      speed_range_str = f'mph {start:02d}-{end:02d}'
      mask = (speed_range[0] <= speed * CV.MS_TO_MPH) & (speed * CV.MS_TO_MPH <= speed_range[1])

      plot_speed = speed[mask]
      plot_angle = angle[mask]
      plot_steer = steer[mask]

      params = None
      if FIT_EACH_PLOT and sum(mask) > 4:
        try:
          params, _ = curve_fit( # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
            _fit_kf,
            np.array([plot_speed, plot_angle]),
            np.array(plot_steer),
            maxfev=9000,
          )
          A, B, C, D, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF = params
        except RuntimeError as e:
          print(e)
          continue

      # print(f'{speed_range_str} ({len(plot_speed)}): {params}')
      angle_points_dict[speed_range_str] = len(plot_speed)
      plt.figure(figsize=(12,8))
      plt.scatter(plot_angle, plot_steer, label=speed_range_str, color='black', s=1.)

      if IS_ANGLE_PLOT:
        _x_ff = np.linspace(-60, 60, res)
      else:
        _x_ff = np.linspace(-3.5, 3.5, res)
      _y_ff = [
          old_feedforward(np.mean(speed_range) * CV.MPH_TO_MS, _i) for _i in _x_ff
      ]
      plt.plot(
          _x_ff,
          _y_ff,
          color='red',
          label='old'
      )
      _y_ff = [
          new_feedforward(np.mean(speed_range) * CV.MPH_TO_MS, _i)
          for _i in _x_ff
      ]
      plt.plot(_x_ff, _y_ff, color='blue', label='new')

      plt.title(speed_range_str)
      plt.legend(loc='lower right')
      if IS_ANGLE_PLOT:
        plt.xlabel('angle (deg)')
        plt.xlim(-55.,55.)
      else:
        plt.xlabel('lateral acceleration (m/s^2)')
        plt.xlim(-3.5,3.5)
      plt.ylabel('steer')
      plt.ylim(-1.5, 1.5)
      plt.grid(axis='x', color='0.95')
      plt.grid(axis='y', color='0.95')
      # plt.xlim(-max(abs(plot_angle)), max(abs(plot_angle)))
      plt.savefig(f'plots/{speed_range_str}.png')
      plt.close()
    
    print(''.join([ "{}:{}{}".format(k,v,'\n' if (ki+1) % 5 == 0 else ', ') for ki,(k,v) in enumerate(sorted(list(angle_points_dict.items()), key=lambda x: x[0]))]))

  if SPEED_PLOTS or ANGLE_PLOTS:
    # Create animations
    cmds = [
      f'rm -rf ~/Downloads/plots',
      # 'convert -delay 40 plots/deg*.png deg-up.gif',
      # 'convert -delay 40 plots/lat*.png deg-up.gif',
      # 'convert -reverse deg-up.gif deg-down.gif',
      # 'convert -loop -1 deg-up.gif deg-down.gif deg.gif',
      'convert -delay 40 plots/mph*.png mph-up.gif',
      'convert -reverse mph-up.gif mph-down.gif',
      'convert -loop -1 mph-up.gif mph-down.gif mph.gif',
      # 'convert -loop -1 deg.gif mph.gif solution.gif',
      'mv *.gif plots/',
      'mv plots ~/Downloads/',
      'rm -f ~/Downloads/plots/deg*.png',
      'rm -f ~/Downloads/plots/lat*.png',
      # 'rm -f ~/Downloads/plots/mph*.png',
      # 'rm -f regularized',
      f'mv ~/Downloads/plots ~/Downloads/plots_{"angle" if IS_ANGLE_PLOT else "torque"}',
      f'cp regularized ~/Downloads/plots_{"angle" if IS_ANGLE_PLOT else "torque"}/regularized'
    ]
    for cmd in cmds:
      print(cmd)
      os.system(cmd)