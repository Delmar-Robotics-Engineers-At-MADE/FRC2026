# MAXMotion Example

## Description

This example shows how to perform both position and velocity closed loop control using MAXMotion on a SPARK motor controller.

### Topics Covered

* Configuring a SPARK motor controller
* Position closed loop control with MAXMotion
* Velocity closed loop control with MAXMotion
* Closed loop slots
* Retrieving encoder data
* Resetting encoder position

## Usage

This example assumes a SPARK MAX with a free spinning NEO. The PID values are tuned for this and should be adjusted if a different setup is being used.

<!-- For information on migrating to SPARK Flex, see our [documentation](). -->

Deploy the program to your roboRIO and load the included `shuffleboard.json` into Shuffleboard. The Shuffleboard layout provides the following:

* A toggle to switch between position and velocity control modes
* Input sliders to set target position or velocity
* Output sliders to show actual position and velocity
* A reset button for the encoder position

### AdvantageScope

To better visualize the motion profile created by MAXMotion, load the included `advantageScope.json` into AdvantageScope. The AdvantageScope layout graphs the target and actual position/velocity.
