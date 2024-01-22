# FuzzyLogicSpeedController_V1
A fuzzy logic controller to control the speed of dc motors based on ultrasonic distance measurement, utilizing the SKFuzzy Python library. 

## Installation and setup

First setup python and virtual environments on your machine, then install the dependencies:

* RPi.GPIO
* Numpy
* Scikit-fuzzy (https://github.com/scikit-fuzzy/scikit-fuzzy)

After that setup your hardware -- this osoyoo kit was used when I did this project: https://osoyoo.com/2020/03/01/use-raspberry-pi-to-control-mecanum-omni-wheel-robot-car/. I used the pins included in that guides instructions so be sure to change the pins in the code if using a different hardware config! This project used an l298n based motor driver, so if that is changed as well the control logic will have to be changed to reflect that.

## Usage

Run the included python file within the terminal in the virtual environment created during setup.


## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

