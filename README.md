# Servo Control system demonstration under Zephyr

This sample presents an application to perform a servo
control by setting a target position and letting an CAN
based servo motor to track it, additionally it offers
PID and ADRC controllers ready to use, tune and live
load.

The graphical view of the system can be done using STMViewer
since the initial target board is a ST Nucleo G474RE which
has a CAN controller.


## Requirements
---------------

### Hardware:
-------------
* MF4005 CAN Bus servo motor (Make sure to checkout a CAN Bus version not the RS485):
    * https://www.aliexpress.com/w/wholesale-mf4005.html?spm=a2g0o.detail.search.0
    * http://en.lkmotor.cn

* ST Nucleo G474RE:
    * https://www.st.com/en/evaluation-tools/nucleo-g474re.html

* A CAN Bus transceiver (A 3.3V):
    *  https://www.aliexpress.com/item/1005003450161614.html?channel=twinner

The Motor can be supplied by a regular 12 VDC power supply, its CANH and CANL
wires should be attached to the CANH and CANL side of the CAN transceiver board
while the CAN TX and CAN RX should be wired to the nucleo board:
* PA11 -> CAN RX;

* PA12 -> CAN TX;

These signals can be found in the Board's CN10 connector on the top side.


### Firmware:
-------------

* Install Zephyr RTOS on your development machine, version 3.4 and 3.5 worked on this firmware:
    * https://docs.zephyrproject.org/latest/develop/getting_started/index.html

* Install STMViewer if you would like to see the graphics while tunning (support for ST SoC only):
    * https://github.com/klonyyy/STMViewer/releases

* Clone this repository.

## Building and Flashing
------------------------

Using this sample with the supported G474RE Nucleo board does not
require special instructions, just use west command as shown below:

```$ west build -p always -b nucleo_g474re path/to/zephyr_control_systems_demo```

Flashing also does not require special steps, just type:

```$ west flash```

## Expected result
------------------

After flashing open your favorite terminal and connect to the Board
you might able to see the Zephyr console and some commands can be
executed (refer main.c to check how the shell commands are implemented):

```
uart:~$ zephyr_motor_control se
  set_reference   set_gains_pid   set_gains_adrc
uart:~$ zephyr_motor_control set_reference 270
uart:~$ zephyr_motor_control set_reference 45
uart:~$ zephyr_motor_control set_reference 90
uart:~$ zephyr_motor_control set_reference 120
uart:~$ zephyr_motor_control set_reference 0
uart:~$ zephyr_motor_control set_reference 10
uart:~$ zephyr_motor_control set_reference 200
uart:~$ zephyr_motor_control set_gains_adrc 0.0005 200 0.1 10 0.2

```

The demonstration firmware implements a digital generic control loop
which receives an interface to the motor plant, the interface supports
other implementations beyond the MF4005, check the implementation and
adapt to your motor.

The generic control loop can receive dynamically a control law object
that takes a form of an generic control law interface, and it was
extended to implement both PID and ADRC controllers, other controllers
like LQR or MPC can be implmented under this interface, check both
pid_control_law.c and adrc_control_law.c implementations.

Additionally there are commands on shell to load ADRC or PID controllers
on thecontrol loop and commands to set their gains, with help of
STMViewer the user can perform the control loops real time, graphical
tunning, saving hours of debug.

The current instances are tuned for the MF4005 plant and it is stable offering
also good tracking and almost zero overshoot under step response.
