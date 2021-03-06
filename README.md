# turu

## Introduction 🎯

A library of PID tuning rules.

## Instalation ⏯️

This is a nano project yet, so you'll must run the chunk of code below to "install" it (at least for now).

    from requests import get   
    url = 'https://raw.githubusercontent.com/gmxavier/turu/main/turu.py' 
    r = get(url)
    with open('turu.py', 'w') as f: 
        f.write(r.text) 
    from turu import *

## Usage 🎛️

Considering the Example 5.7A from [here](https://ia802909.us.archive.org/32/items/process-control-a-first-course-with-matlab/Process%20Control%20A%20First%20Course%20with%20MATLAB.pdf#page=128). What would be the PID controller settings using the Ziegler Nichols rule?

    # Ziegler-Nichols tuning settings as gains (proportional, integral and derivative)
    Kp, Ki, Kd = ziegler_nichols(K=1.25, tau=4, theta=0.9, type_of_controller='PID')
    # tuning settings as proportional gain, integral time and derivative time
    [Kp, Kp/Ki, Kd/Kp]
    [4.266666666666667, 1.8, 0.45]

## Contributing 🧱

If you ❤️ Python and know a PID tuning rule that is not [here](http://cyxtp.ucoz.ru/pdf/Aidan_O_Dwyer_Handbook_of_PI_and_PID_Controller_Tuning_Rules.pdf), please code it and make a pull request.

If you ❤️ Python and need a PID tuning rule that is [here](http://cyxtp.ucoz.ru/pdf/Aidan_O_Dwyer_Handbook_of_PI_and_PID_Controller_Tuning_Rules.pdf), but not in `turu` yet, please code it and make a pull request.

If you ❤️ this project and found an bug, need a feature or have a suggestion, we kindly ask you to open an issue.
