# Introduction
You may have noticed that in this project subsystems consist of a package instead of one file. This is because the project uses AdvantageKit ([docs](https://docs.advantagekit.org/)).
# Why?
AdvantageKit allows us to do some neat stuff:
- Log states of subsystems, available on NetworkTables at runtime as well as a log file
	- Log files contain inputs and outputs
- Log files can be replayed with new code by simulating what the robot would do if it was given the inputs from that session
	- This is really helpful to see if a bug is fixed by confirming whether unexpected behavior is eliminated
	- Also really helpful for rapid prototyping; for instance, trust values for cameras can be fine-tuned without needing access to the robot and needing to set up the driver station, radio, etc...
- Log files can be viewed after a match to show where the robot was and what it was doing
	- Absolutely critical when it comes to visualizing what drivers were experiencing during a match. For instance, if the robot unexpectedly jumps around, a look in the log files could confirm that the camera placement is off and the robot thinks it is in a different place than it really is
- Have separate behaviors for simulated vs. real instances
	- In a simulation, real motors cannot be used, so one can write a new implementation of a subsystem IO layer acting as a compatibility layer for testing
# How?
AdvantageKit's paradigm is that each subsystem is divided into two layers: common logic, and hardware abstraction layers.
## Common Logic
This is where everything that doesn't require interfacing directly with hardware should live. This is where calculations should be done, where we hold our command factories, etc.
## Hardware Abstraction
### Interfaces
Each subsystem has an interface, usually titled something like `[SubsystemName]IO`. In here are a few things: a class with the `@AutoLog` annotation consisting of the various inputs your hardware shares, like position, velocity, target position, voltage, current, etc.; and default functions common to all implementations of this IO interface, as well as a method `UpdateInputs` that takes an instance of the inputs class.
### Implementations
A typical subsystem will have two implementations of this interface: one for a real instance of the robot, and one for a simulated instance. Each of the methods are implemented with logic that is unique to the implementation's specific hardware implementation.
