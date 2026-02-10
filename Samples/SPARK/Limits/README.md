# Limits Example

## Description

This example shows how to use both physical and soft limits on a SPARK motor controller.

### Topics Covered

* Configuring a SPARK motor controller
* Stopping the motor with a limit switch
* Stopping the motor with a soft limit
* Retrieving encoder data
* Resetting encoder position

## Usage

This example assumes a SPARK MAX with a free spinning NEO and both forward and reverse limit switches connected through the data port.

<!-- For information on migrating to SPARK Flex, see our [documentation](). -->

Deploy the program to your roboRIO and load the included `shuffleboard.json` into Shuffleboard. The Shuffleboard layout provides the following:

* A toggle to switch between running the motor in the forward and reverse direction
* Boolean boxes to display if either the forward or reverse limit switch has been reached
* An output slider for the applied output to display the motor's direction
* An output slider for the encoder position to display the soft limits' behavior
