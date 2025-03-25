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
package frc.robot

import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.generated.TunerConstants
import frc.robot.util.Elastic
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.math.roundToInt

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null
    private val robotContainer: RobotContainer

    init {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }

        // Set up data receivers & replay source
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.SIM ->         // Running a physics simulator, log to NT
                Logger.addDataReceiver(NT4Publisher())

            Constants.Mode.REPLAY -> {
                // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }
        //Logger.registerURCL(URCL.startExternal())

        // Start AdvantageKit logger
        Logger.start()

        // Check for valid swerve config
        val modules = arrayOf<SwerveModuleConstants<*, *, *>>(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        )
        HAL.report(
            FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin
        )
        for (constants in modules) {
            if ((constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated) || (constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated)) {
                throw RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers."
                )
            }
        }
        WebServer.start(5800, Filesystem.getDeployDirectory().path) // For dashboard files

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = RobotContainer()

        // Schedule the warmup command. Significantly speeds up pathfinding after the first run.
        PathfindingCommand.warmupCommand().withName("Pathfinding Warmup").schedule()

        // Silence joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true)
    }

    companion object {
        @JvmStatic
        var alliance = if (DriverStation.getAlliance().isPresent) DriverStation.getAlliance()
            .get() else Alliance.Red

        fun reportError(error: String) {
            DriverStation.reportError(error, false)
            Elastic.sendNotification(
                Elastic.Notification(
                    Elastic.Notification.NotificationLevel.ERROR, "Error", error
                )
            )
        }

        fun reportWarning(warning: String) {
            DriverStation.reportWarning(warning, false)
            Elastic.sendNotification(
                Elastic.Notification(
                    Elastic.Notification.NotificationLevel.WARNING, "Warning", warning
                )
            )
        }

        fun reportInfo(info: String) {
            DriverStation.reportWarning(info, false)
            Elastic.sendNotification(
                Elastic.Notification(
                    Elastic.Notification.NotificationLevel.INFO, "Info", info
                )
            )
        }
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        //Threads.setCurrentThreadPriority(true, 99)
        CommandScheduler.getInstance().run()
        //Threads.setCurrentThreadPriority(false, 10)
        for (i in 0..5) run {
            Logger.recordOutput(
                "Controller/Axis$i",
                robotContainer.markIVController.getRawAxis(i)
                    .let { if (it < 0.0) it * 127.0 else it * 128.0 }.let { it + 127.0 }.roundToInt()
                    .toString(2).padStart(8, '0')
            )

            Logger.recordOutput(
                "Controller/RawAxis$i",
                robotContainer.markIVController.getRawAxis(i)
                    .let { if (it < 0.0) it * 127.0 else it * 128.0 }.let { it + 127.0 }.roundToInt()
            )

            Logger.recordOutput(
                "Controller/ReallyRawAxis$i",
                robotContainer.markIVController.getRawAxis(i)
                    .let { if (it < 0.0) it * 127.0 else it * 128.0 }.roundToInt()
            )
            Logger.recordOutput(
                "Controller/ReallyReallyRawAxis$i",
                robotContainer.markIVController.getRawAxis(i)
            )
        }
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        robotContainer.stopQueue()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        autonomousCommand = robotContainer.autonomousCommand

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand!!.schedule()
        }
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto()
    }

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()
        robotContainer.displaySimFieldToAdvantageScope()
    }


}
