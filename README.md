# FRC 2025 Robot Project

## Overview

This project is designed for the FRC 2025 season, utilizing Kotlin and Java for robot control and
telemetry. The project is structured to follow the Command-based programming paradigm, which is a
declarative approach to robot programming. The project includes various subsystems, commands, and
utilities to control a swerve drive robot.

## Project Structure

The project is organized into several key directories and files:

- `src/main/java/frc/robot/`: Contains the main robot code, including subsystems, commands, and
  utilities.
    - `Constants.kt`: Defines various constants used throughout the project.
    - `Robot.kt`: The main robot class, extending `TimedRobot`.
    - `RobotContainer.kt`: Sets up the robot's subsystems, commands, and button bindings.
    - `Telemetry.kt`: Handles telemetry data publishing and logging.
    - `Main.kt`: The entry point of the robot application.
    - `subsystems/`: Contains subsystem classes, such as `CommandSwerveDrivetrain` and
      `ExampleSubsystem`.
    - `commands/`: Contains command classes, such as `ExampleCommand` and `Autos`.

## Major Libraries Used

- **WPILib**: The core library for FRC robot programming, providing support for hardware interfaces,
  control algorithms, and more.
- **CTRE Phoenix**: A library from Cross The Road Electronics for controlling their motor
  controllers and sensors.
- **PathPlanner**: A library for autonomous path planning and following.

## Key Components

### Subsystems

- **CommandSwerveDrivetrain**: Extends the CTRE `SwerveDrivetrain` class and implements the
  `Subsystem` interface for swerve drive control.
- **ExampleSubsystem**: A simple example subsystem demonstrating basic structure and functionality.

### Commands

- **ExampleCommand**: An example command that interacts with `ExampleSubsystem`.
- **Autos**: Manages autonomous commands and provides a chooser for selecting different autonomous
  modes.

### Telemetry

- **Telemetry**: Handles publishing and logging of telemetry data, including robot pose, speeds, and
  module states.

### Constants

- **Constants**: Defines various constants used throughout the project, such as operator constants
  and physical properties of the robot.

## Getting Started

1. **Clone the repository**:
   ```sh
   git clone https://github.com/BearBots6964/FRC2025.git
   cd FRC2025
   ```
> [!IMPORTANT]
> Make sure you have Git installed on your system, along with the [2025 WPILib release](https://github.com/wpilibsuite/allwpilib/releases).

2. **Open the project in IntelliJ IDEA**:
    - Open IntelliJ IDEA.
    - Select "Open" and navigate to the project directory.
> [!NOTE]
> Other integrated development environments such as Visual Studio Code,
> Emacs, Neovim, and Helix *should* work, but have not been tested with this codebase.
> 
(Visual Studio Code, or at least the WPILib distribution of it,
is the only officially supported IDE. We use IntelliJ IDEA for development because it's better.)

> [!IMPORTANT]
> Ensure you have the FRC plugin installed in IntelliJ IDEA.

3. **Build the project**:
    - Use the Gradle tool window to build the project.
    - Ensure all dependencies are downloaded and the project compiles successfully.

4. **Deploy to the robot**:
    - Connect to the robot's network.
    - Use the WPILib tools to deploy the code to the robot.
> [!NOTE]
> If the Gradle tasks do not appear in the top right dropdown next to the build and debug buttons,
> you may need to add the FRC Gradle shortcuts by hitting Shift twice
> and typing "Create Run/Debug Configurations."

## Contributing
See [CONTRIBUTING.md](doc/CONTRIBUTING.md) for information on how to contribute to this project.