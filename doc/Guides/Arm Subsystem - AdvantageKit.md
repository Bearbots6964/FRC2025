Let's use the example of an arm subsystem, and break it down into individual processes.

> [!IMPORTANT] 
> Some knowledge of controls engineering is required in order to proceed. **To learn even just the basics is a huge undertaking**, and **will** take a fairly long amount of time to fully understand. This isn't a guide that can be followed in one afternoon.
> 
> An incredibly in-depth guide can be found [here](https://controls-in-frc.link/). It's likely you will have to ask someone for help understanding all of this, but don't be afraid of asking; this is graduate student-level knowledge and is not something taught in high schools.

# Identify Requirements
In our arm, we have a motor and an encoder. We use the encoder's output to determine what to do with the motor in order to get to some position. Our goal with this subsystem is to have the ability to command the arm to go to a position, and for the arm to reach that position within a reasonable tolerance.
## Physical Requirements
- An arm
- A motor connected to a motor controller (we'll use a Kraken as an example)
- An encoder (let's have this be a CANcoder)
## Technical Requirements
- Knowledge of the arm's physical properties
	- This requires measurements either in the CAD (which needs to be accurate) or experimentally taken
- Motor and encoder configuration
	- CAN ID, current limits, etc
	- See the [[TalonFX Motor Configuration Template]] to get an idea of how to configure motors in code. The same concept can be applied to the CANcoder, but it requires different configuration objects.
	- See [[Bringing Up a Closed-Loop Motor]] for instructions on how to bring everything up.
# Preliminary Tasks
Before we can start writing code, we need to calculate some constants and ensure we have a basic understanding of what we're doing.
## Calculations
[ReCalc](https://www.reca.lc/arm) has our back when it comes to calculations. Input your arm setup into their respective fields. You can view some documentation by clicking the "Docs" button near the bottom of the page. You'll want to write these down somewhere.
> [!TIP]
> If you don't have an accurate CAD available and can't measure physical properties, you'll have to use SysID. Details can be found [here](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html). Later in this guide you'll learn how to implement the required code.
## Set Up Your Environment
Make sure you have the latest version of WPILib installed. You can use an [AdvantageKit-provided template project](https://docs.advantagekit.org/getting-started/installation/) for this, although you may need to do some work if you want your code to be in Kotlin (although at the time of writing the IntelliJ FRC plugin is out-of-date anyway, so you might as well just use the template project and convert it to Kotlin).
It's probably best to go with a template that corresponds to your robot's actual drive system. The AdvantageKit documentation will guide you through everything you need to know to set up your template.
### Vendordeps
Run the following command in a terminal:
```sh
./gradlew vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
```
> [!TIP]
> This can also be done in IntelliJ IDEA in its built-in Gradle interface. This may bypass some issues when using the command line.
> - Run Anything (hit SHIFT twice) > `Execute Gradle Task` > paste in the above terminal command but remove the `./gradlew`
# Get Started
> [!NOTE]
> If you want to use Kotlin, take a look at [[Use Kotlin with AdvantageKit]]. Be aware it may not work and is highly finnicky.

## Create Your Files
Under `src/main/java/frc/robot/subsystems` (corresponding to package `frc.robot.subsystems`), create a new package called `arm`. Create the following Java files under that directory:
- `Arm.java`
- `ArmIO.java`
- `ArmIOTalonFX.java`
- `ArmIOSim.java`
Additionally, ensure you have a `Constants` file somewhere in your project.