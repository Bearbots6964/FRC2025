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
import edu.wpi.first.hal.FRCNetComm.tInstances.*
import edu.wpi.first.hal.FRCNetComm.tResourceType.*
import edu.wpi.first.hal.HAL
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null
    private val robotContainer: RobotContainer
    private var firstDisable = false

    init {
        println("[Robot:0ms] Robot class initialized")
        val initializeTime = Timer.getFPGATimestamp()
        var stopTime = Timer.getFPGATimestamp()
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
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Logger metadata recorded in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")
        stopTime = Timer.getFPGATimestamp()
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
                Logger.addDataReceiver(WPILOGWriter(addPathSuffix(logPath, "_sim")))
            }
        }
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Logger data receivers set up in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")
        //Logger.registerURCL(URCL.startExternal())

        stopTime = Timer.getFPGATimestamp()
        // Start AdvantageKit logger
        Logger.start()
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Logger started in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")

        stopTime = Timer.getFPGATimestamp()
        // Check for valid swerve config
        val modules = arrayOf<SwerveModuleConstants<*, *, *>>(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        )

        HAL.report(
            kResourceType_Language, kLanguage_Kotlin
        )
        HAL.report(
            kResourceType_Framework, kFramework_AdvantageKit
        )
        HAL.report(
            kResourceType_RobotDrive, kRobotDriveSwerve_AdvantageKit
        )
        HAL.report(kResourceType_Dashboard, kDashboard_Elastic)
        HAL.report(kResourceType_Dashboard, kDashboard_AdvantageScope)
        HAL.report(kResourceType_PDP, kPDP_REV)
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] HAL reports sent in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")

        stopTime = Timer.getFPGATimestamp()
        for (constants in modules) {
            if ((constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated) || (constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated)) {
                throw RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers."
                )
            }
        }
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Swerve config checked in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")
        stopTime = Timer.getFPGATimestamp()
        WebServer.start(5800, Filesystem.getDeployDirectory().path) // For dashboard files
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Web server started in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")
        stopTime = Timer.getFPGATimestamp()

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = RobotContainer()
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] RobotContainer initialized in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")
        stopTime = Timer.getFPGATimestamp()

        // Schedule the warmup command. Significantly speeds up pathfinding after the first run.
        PathfindingCommand.warmupCommand().withName("Pathfinding Warmup").schedule()
        println("[Robot:${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms] Pathfinding warmup command scheduled in ${"%.3f".format((Timer.getFPGATimestamp() - stopTime) * 1000.0)}ms")

        // Silence joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true)

        println("Robot initialization took ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
    }

    companion object {

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

        var inTeleop = false
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
        robotContainer.updateHmiAlgae()
        if (robotContainer.enableEmergencyDashboard) {
            robotContainer.emergencyDashboardSetup()
            robotContainer.enableEmergencyDashboard = false
        }
        //Threads.setCurrentThreadPriority(false, 10)
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        robotContainer.stopQueue()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        inTeleop = false
        autonomousCommand = robotContainer.autonomousCommand
        firstDisable = true

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
        robotContainer.fixArm()
        Companion.inTeleop = true
        robotContainer.disableAuto()
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

    /**
     * Adds a suffix to the given path (e.g. "test.wpilog" -> "test_sim.wpilog").
     *
     *
     *
     * If the input path already contains the suffix, an index will be added
     * instead.
     */
    fun addPathSuffix(path: String, suffix: String): String {
        val dotIndex = path.lastIndexOf(".")
        if (dotIndex == -1) {
            return path
        }
        val basename = path.substring(0, dotIndex)
        val extension = path.substring(dotIndex)
        if (basename.endsWith(suffix)) {
            return basename + "_2" + extension
        } else if (basename.matches((".+" + suffix + "_[0-9]+$").toRegex())) {
            val splitIndex = basename.lastIndexOf("_")
            val index = basename.substring(splitIndex + 1).toInt()
            return (basename.substring(0, splitIndex) + "_" + (index + 1).toString() + extension)
        } else {
            return basename + suffix + extension
        }
    }
}
