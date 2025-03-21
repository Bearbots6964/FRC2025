// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.commands

import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Robot
import frc.robot.subsystems.drive.Drive
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.*
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.withSign

object DriveCommands {
    private const val DEADBAND = 0.1
    private const val ANGLE_KP = 5.0
    private const val ANGLE_KD = 0.4
    private const val ANGLE_MAX_VELOCITY = 8.0
    private const val ANGLE_MAX_ACCELERATION = 20.0
    private const val FF_START_DELAY = 2.0 // Secs
    private const val FF_RAMP_RATE = 0.1 // Volts/Sec
    private const val WHEEL_RADIUS_MAX_VELOCITY = 0.25 // Rad/Sec
    private const val WHEEL_RADIUS_RAMP_RATE = 0.05 // Rad/Sec^2

    private fun getLinearVelocityFromJoysticks(x: Double, y: Double): Translation2d {
        // Apply deadband
        var linearMagnitude = MathUtil.applyDeadband(hypot(x, y), DEADBAND)
        val linearDirection = Rotation2d(atan2(y, x))

        // Square magnitude for more precise control
        linearMagnitude *= linearMagnitude

        // Return new linear velocity
        return Pose2d(Translation2d(), linearDirection)
            .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
            .translation
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    fun joystickDrive(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        omegaSupplier: DoubleSupplier,
    ): Command {
        return Commands.run(
            {
                // Get linear velocity
                val linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.asDouble, ySupplier.asDouble)

                // Apply rotation deadband
                var omega = MathUtil.applyDeadband(omegaSupplier.asDouble, DEADBAND)

                // Square rotation value for more precise control
                omega = (omega * omega).withSign(omega)

                // Convert to field relative speeds & send command
                val speeds =
                    ChassisSpeeds(
                        linearVelocity.x * drive.maxLinearSpeedMetersPerSec,
                        linearVelocity.y * drive.maxLinearSpeedMetersPerSec,
                        omega * drive.maxAngularSpeedRadPerSec
                    )
                val isFlipped =
                    Robot.alliance == Alliance.Red
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        if (isFlipped)
                            drive.rotation.plus(Rotation2d(Math.PI))
                        else
                            drive.rotation
                    )
                )
            },
            drive
        ).withName("Joystick Drive")
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    fun joystickDriveAtAngle(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        rotationSupplier: Supplier<Rotation2d>
    ): Command {
        // Create PID controller

        val angleController =
            ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
            )
        angleController.enableContinuousInput(-Math.PI, Math.PI)

        // Construct command
        return Commands.run(
            {
                // Get linear velocity
                val linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.asDouble, ySupplier.asDouble)

                // Calculate angular speed
                val omega =
                    angleController.calculate(
                        drive.rotation.radians, rotationSupplier.get().radians
                    )

                // Convert to field relative speeds & send command
                val speeds =
                    ChassisSpeeds(
                        linearVelocity.x * drive.maxLinearSpeedMetersPerSec,
                        linearVelocity.y * drive.maxLinearSpeedMetersPerSec,
                        omega
                    )
                val isFlipped =
                    Robot.alliance == Alliance.Red
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        if (isFlipped)
                            drive.rotation.plus(Rotation2d(Math.PI))
                        else
                            drive.rotation
                    )
                )
            },
            drive
        ) // Reset PID controller when command starts

            .beforeStarting({ angleController.reset(drive.rotation.radians) })
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     *
     * This command should only be used in voltage control mode.
     */
    fun feedforwardCharacterization(drive: Drive): Command {
        val velocitySamples: MutableList<Double> = LinkedList()
        val voltageSamples: MutableList<Double> = LinkedList()
        val timer = Timer()

        return Commands.sequence( // Reset data
            Commands.runOnce(
                {
                    velocitySamples.clear()
                    voltageSamples.clear()
                }),  // Allow modules to orient

            Commands.run(
                {
                    drive.runCharacterization(0.0)
                },
                drive
            )
                .withTimeout(FF_START_DELAY),  // Start timer

            Commands.runOnce({ timer.restart() }),  // Accelerate and gather data

            Commands.run(
                {
                    val voltage = timer.get() * FF_RAMP_RATE
                    drive.runCharacterization(voltage)
                    velocitySamples.add(drive.ffCharacterizationVelocity)
                    voltageSamples.add(voltage)
                },
                drive
            ) // When cancelled, calculate and print results

                .finallyDo(
                    Runnable {
                        val n = velocitySamples.size
                        var sumX = 0.0
                        var sumY = 0.0
                        var sumXY = 0.0
                        var sumX2 = 0.0
                        for (i in 0..<n) {
                            sumX += velocitySamples[i]
                            sumY += voltageSamples[i]
                            sumXY += velocitySamples[i] * voltageSamples[i]
                            sumX2 += velocitySamples[i] * velocitySamples[i]
                        }
                        val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                        val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                        val formatter: NumberFormat = DecimalFormat("#0.00000")
                        println("********** Drive FF Characterization Results **********")
                        println("\tkS: " + formatter.format(kS))
                        println("\tkV: " + formatter.format(kV))
                    })
        )
    }

    /** Measures the robot's wheel radius by spinning in a circle.  */
    fun wheelRadiusCharacterization(drive: Drive): Command {
        val limiter = SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE)
        val state = WheelRadiusCharacterizationState()

        return Commands.parallel( // Drive control sequence
            Commands.sequence( // Reset acceleration limiter
                Commands.runOnce(
                    {
                        limiter.reset(0.0)
                    }),  // Turn in place, accelerating up to full speed

                Commands.run(
                    {
                        val speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)
                        drive.runVelocity(ChassisSpeeds(0.0, 0.0, speed))
                    },
                    drive
                )
            ),  // Measurement sequence

            Commands.sequence( // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),  // Record starting measurement

                Commands.runOnce(
                    {
                        state.positions = drive.wheelRadiusCharacterizationPositions
                        state.lastAngle = drive.rotation
                        state.gyroDelta = 0.0
                    }),  // Update gyro delta

                Commands.run(
                    {
                        val rotation = drive.rotation
                        state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
                        state.lastAngle = rotation
                    }) // When cancelled, calculate and print results

                    .finallyDo(
                        Runnable {
                            val positions = drive.wheelRadiusCharacterizationPositions
                            var wheelDelta = 0.0
                            for (i in 0..3) {
                                wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                            }
                            val wheelRadius =
                                (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta

                            val formatter: NumberFormat = DecimalFormat("#0.000")
                            println(
                                "********** Wheel Radius Characterization Results **********"
                            )
                            println(
                                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians"
                            )
                            println(
                                "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians"
                            )
                            println(
                                ("\tWheel Radius: "
                                        + formatter.format(wheelRadius)
                                        + " meters, "
                                        + formatter.format(Units.metersToInches(wheelRadius))
                                        + " inches")
                            )
                        })
            )
        )
    }


    fun pathfindThenFollowPath(drive: Drive, path: PathPlannerPath): Command {
        return drive.pathfindThenFollowPath(path)
    }

    private class WheelRadiusCharacterizationState {
        var positions: DoubleArray = DoubleArray(4)
        var lastAngle: Rotation2d = Rotation2d()
        var gyroDelta: Double = 0.0
    }
}
