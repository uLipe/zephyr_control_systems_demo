# FoC Motor control Example

This sample presents an applicatoin to perform a Field Oriented 
Control (FoC) to command Brushless motors such PMSM or BLDC.


## Overview
-----------

The FoC pipeline in this sample uses the voltage mode command,
that is it, no current control in closed loop, instead the pair
of voltages, direct and quadrature are applied directly to the
FoC engine, then converted to a 3-phase stator voltages.

The pipeline is triggered each time the motor rotor position
is measured, then the other node chains are executed, one for 
execute FoC followed by the PWM Inverter update.


## Requirements
---------------

To experiment the motor control is necessary a power electronics interface
that supports three PWM signals on its input. This sample was tested using 
the ST Microelectronics IHM07M1 brushless motor interface development kit
plus a Nucleo G474RE board, altough any nucleo board that has support for 
PWM and I2C from Zephyr would work.

Also besides a brushless motor, it is necessary a magnetic encoder board 
like AS5600 (or AS5408 which is register compatible) with I2C interface.
This FoC controller is a voltage mode, that is it, a rotor position sensor 
like this magnetic encoder is required, it can be attached to any I2C 
available on the MCU, for this test the I2C1 under PB8 and PB9 pins 
were selected in the Nucleo board.

For the PWM, the test was made using the pins PA8, PA9 and PA10, these 
pins output the PWM signlas from TIM1, respectively the channels 1 to 3.
Also the driver on the power board have 3 enable signals that need to be
tied to high, in the test they are connected on PC10 to PC12. More 
information can be obtained checking the sample overlay file for this
application. 

In src/main.c the user may select the SAMPLE_DC_LINK_VOLTAGE_MV, depending
on the voltage applied in the power stage, which is typically 12V, The 
constant SAMPLE_MOTOR_POLE_PAIRS is motor dependent and needs to be checked
in the target motor documentation, values amoung 7 and 14 pole pairs are 
very common.


## Building and Flashing
------------------------

Using this sample with the supported G474RE Nucleo board does not 
require special instructions, just use west command as shown below:

```$ west build -p always -b nucleo_g474re path/to/zephyr_foc_controller```

Flashing also does not require special steps, just type:

```$ west flash```


## Expected result
------------------

After flashing the code the shell will become available and 
you can type step_foc set <Uq> <Ud> command to set the voltage direct 
and quadrature commands. Uq is the quadrature voltage used mainly 
to perform speed variation, Ud is used as field weakening reducing 
the motor shaft torque in exchange for higher speeds.

Reminder Uq and Ud ranges are normalized between -1.0 and +1.0, typing the command below 
will result in immediate motor spinning.

```$ foc set -0.5 0.0```
