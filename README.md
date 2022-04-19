# turu

## Introduction

A Python collection of PID tuning rules.

## Instalation

This is a nano project (at least for now), so the easier way to use it is running the code below:

    from requests import get   
    url = 'https://raw.githubusercontent.com/gmxavier/turu/main/turu.py' 
    r = get(url)
    with open('turu.py', 'w') as f: 
        f.write(r.text) 
    from turu import *

## Usage

Considering the Example 5.7A from [here](https://ia802909.us.archive.org/32/items/process-control-a-first-course-with-matlab/Process%20Control%20A%20First%20Course%20with%20MATLAB.pdf#page=128)).

What would be the PID controller settings using the Ziegler Nichols rule?

    Kp, Ki, Kd = ziegler_nichols(K=1.25, tau=4, theta=0.9, type_of_controller='PID'); [Kp, Kp/Ki, Kd/Kp]
    [4.266666666666667, 1.8, 0.45]
