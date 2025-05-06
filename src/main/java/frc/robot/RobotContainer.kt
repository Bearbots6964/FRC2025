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

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.VisionConstants.robotToBackLeftCamera
import frc.robot.Constants.VisionConstants.robotToBackRightCamera
import frc.robot.Constants.VisionConstants.robotToFrontLeftCamera
import frc.robot.Constants.VisionConstants.robotToFrontRightCamera
import frc.robot.commands.*
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.arm.*
import frc.robot.subsystems.climber.*
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.elevator.Elevator
import frc.robot.subsystems.elevator.ElevatorIO
import frc.robot.subsystems.elevator.ElevatorIOTalonFX
import frc.robot.subsystems.intake.AlgaeIntake
import frc.robot.subsystems.intake.AlgaeIntakeIO
import frc.robot.subsystems.intake.AlgaeIntakeIOSparkMax
import frc.robot.subsystems.vision.*
import frc.robot.util.CommandQueue
import frc.robot.util.Elastic
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Supplier
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.roundToInt


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@Suppress("GrazieInspection")
class RobotContainer {
    // Subsystems
    private var drive: Drive
    private var vision: Vision
    private var arm: Arm
    private var elevator: Elevator
    private var algaeIntake: AlgaeIntake
    private var clawIntake: ClawIntake
    private var climber: Climber

    private var driveSimulation: SwerveDriveSimulation? = null

    // Controller
    private val driveController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val hmi = CommandXboxController(2)

    private var nextReef = PathfindingFactories.Reef.A
    private var nextStation = PathfindingFactories.CoralStationSide.LEFT
    private var nextPosition = Constants.SuperstructureConstants.SuperstructureState.L4
    private var nextAlgaePosition = PathfindingFactories.Reef.AB_ALGAE
    private var grabAlgaeToggle = false

    private var coralStatus = CoralStatus.NONE
    private var algaeStatus = AlgaeStatus.NONE
    private var bargePosition = BargePositions.NONE

    private val driveQueue: CommandQueue = CommandQueue().withCallback {
        Commands.race(
            Commands.waitSeconds(0.5), Commands.run({
                driveController.setRumble(
                    GenericHID.RumbleType.kBothRumble, 1.0
                )
            }).finallyDo(
                Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) })
        )
    }
    private val superstructureQueue: CommandQueue = CommandQueue().withCallback {
        Commands.race(
            Commands.waitSeconds(0.5), Commands.run({
                driveController.setRumble(
                    GenericHID.RumbleType.kBothRumble, 1.0
                )
            }).finallyDo(
                Runnable { driveController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) })
        )
    }.withName("Superstructure Queue")

    // Dashboard inputs
    private lateinit var autoChooser: LoggedDashboardChooser<Command>
    private var bargeChooser: LoggedDashboardChooser<BargePositions>

    private val driveTranslationalControlSupplier: Supplier<Translation2d> =
        Supplier<Translation2d> {
            var xControl: Double = -driveController.leftY
            var yControl: Double = -driveController.leftX
            val magnitude = hypot(xControl, yControl)
            if (magnitude > 1) {
                xControl /= magnitude
                yControl /= magnitude
            } else if (magnitude > 1e-6) {
                val scalar = (MathUtil.applyDeadband(magnitude, .06) / magnitude).pow(2.0)
                xControl *= scalar
                yControl *= scalar
            }
            Translation2d(xControl, yControl)
        }

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        println("┌  [RobotContainer] Initializing...")
        val initializeTime: Double = Timer.getFPGATimestamp()
        driveQueue.name = ("Auto Queue")
        println("╞╦ [RobotContainer] Initializing subsystems at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Real robot, instantiate hardware IO implementations
                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                    ModuleIOTalonFXReal(TunerConstants.FrontRight),
                    ModuleIOTalonFXReal(TunerConstants.BackLeft),
                    ModuleIOTalonFXReal(TunerConstants.BackRight)
                ) { _: Pose2d? -> }
                this.vision = Vision(
                    drive, VisionIOPhotonVision(
                        Constants.VisionConstants.frontRightCameraName, robotToFrontRightCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.backRightCameraName, robotToBackRightCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.frontLeftCameraName, robotToFrontLeftCamera
                    ), VisionIOPhotonVision(
                        Constants.VisionConstants.backLeftCameraName, robotToBackLeftCamera
                    )
                )
                clawIntake = ClawIntake(
                    ClawIntakeIOSparkMax(Constants.SuperstructureConstants.ClawIntakeConstants.sparkConfig)
                )

                elevator = Elevator(
                    ElevatorIOTalonFX(
                        Constants.SuperstructureConstants.ElevatorConstants.leftMotorConfig,
                        Constants.SuperstructureConstants.ElevatorConstants.rightMotorConfig
                    )
                )

                arm = Arm(
                    ArmIOTalonFX(
                        Constants.SuperstructureConstants.ArmConstants.talonConfig
                    ), elevator.elevatorLigament
                )
                algaeIntake = AlgaeIntake(
                    AlgaeIntakeIOSparkMax(
                        Constants.AlgaeIntakeConstants.armConfig,
                        Constants.AlgaeIntakeConstants.intakeConfig
                    )
                )
                climber = Climber(
                    WinchIOTalonFX(Constants.ClimberConstants.winchMotorConfig),
                    ClimberPivotIOTalonFX(Constants.ClimberConstants.pivotMotorConfig)
                )
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                    SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.0, 3.0, Rotation2d()))
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)
                drive = Drive(
                    GyroIOSim(driveSimulation!!.gyroSimulation), ModuleIOTalonFXSim(
                        TunerConstants.FrontLeft, driveSimulation!!.modules[0]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.FrontRight, driveSimulation!!.modules[1]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.BackLeft, driveSimulation!!.modules[2]
                    ), ModuleIOTalonFXSim(
                        TunerConstants.BackRight, driveSimulation!!.modules[3]
                    )
                ) { robotPose: Pose2d? -> driveSimulation!!.setSimulationWorldPose(robotPose) }
                vision = Vision(
                    drive, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.frontRightCameraName, robotToFrontRightCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.backRightCameraName, robotToBackRightCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.frontLeftCameraName, robotToFrontLeftCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose }, VisionIOPhotonVisionSim(
                        Constants.VisionConstants.backLeftCameraName, robotToBackLeftCamera
                    ) { driveSimulation!!.simulatedDriveTrainPose })
                clawIntake = ClawIntake(object : ClawIntakeIO {})
                elevator = Elevator(object : ElevatorIO {})
                climber = Climber(object : WinchIO {}, object : ClimberPivotIO {})

                arm = Arm(
                    ArmIOTalonFXSim(

                    ), elevator.elevatorLigament
                )
                algaeIntake = AlgaeIntake(object : AlgaeIntakeIO {})
            }

            else -> {
                // Replayed robot, disable IO implementations
                drive = Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {}) { _: Pose2d? -> }
                vision = Vision(drive, object : VisionIO {}, object : VisionIO {})

                elevator = Elevator(object : ElevatorIO {})
                arm = Arm(object : ArmIO {}, elevator.elevatorLigament)
                clawIntake = ClawIntake(object : ClawIntakeIO {})
                algaeIntake = AlgaeIntake(object : AlgaeIntakeIO {})
                climber = Climber(object : WinchIO {}, object : ClimberPivotIO {})
            }
        }
        println("╞╩ [RobotContainer] Subsystems initialized at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
        addNamedCommands()
        println("├  [RobotContainer] Named commands initialized at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
        setUpAutoChooser()
        println("├  [RobotContainer] Auto chooser initialized at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")

        // Configure the button bindings
        configureButtonBindings()
        println("├  [RobotContainer] Button bindings configured at ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")

        if (climber.position > 40.0) elevator.goToPosition(40.0)
            .deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }))
            .andThen(SuperstructureCommands.algaeIntakeWithoutSafety(elevator, arm, climber))
            .schedule()

        bargeChooser = LoggedDashboardChooser("Barge Position")
        bargeChooser.addOption("Left", BargePositions.LEFT)
        bargeChooser.addOption("Middle", BargePositions.MIDDLE)
        bargeChooser.addOption("Right", BargePositions.RIGHT)
        bargeChooser.addDefaultOption("None", BargePositions.NONE)

        println("└  [RobotContainer] Initialized in ${"%.3f".format((Timer.getFPGATimestamp() - initializeTime) * 1000.0)}ms")
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        // <editor-fold desc="Drive Controller">
        // Default command, normal field-relative drive
        drive.defaultCommand = DriveCommands.joystickDrive(
            drive,
            { -driveController.leftY },
            { -driveController.leftX },
            { -driveController.rightX })

        // Default commands for elevator and arm
        elevator.defaultCommand = elevator.stop()
        arm.defaultCommand = arm.stop()
        climber.defaultCommand = climber.moveClimberToIntakePosition()
        clawIntake.defaultCommand = clawIntake.stop()
        // Lock to 0° when A button is held
        driveController.a().whileTrue(
            Commands.run({ drive.stopWithX() }, drive)
        )

        // Switch to X pattern when X button is pressed

        // Pathfinding commands

        // Reduced speed drive when B button is pressed
        driveController.b().onTrue(
            algaeCycle()
        )
        // Arm control for left trigger
        driveController.leftTrigger().whileTrue(
            DriveCommands.joystickDrive(
                drive,
                { -driveController.leftY * 0.25 },
                { -driveController.leftX * 0.25 },
                { -driveController.rightX * 0.25 })
        )

        // Reset gyro / odometry
        val resetGyro = if (Constants.currentMode == Constants.Mode.SIM) Runnable {
            drive.pose = driveSimulation!!.simulatedDriveTrainPose
        } else Runnable {
            drive.pose = Pose2d(drive.pose.translation, Rotation2d())
        }
        driveController.start().onTrue(
            runOnce(resetGyro, drive).ignoringDisable(true)
        )
        // reset states
        driveController.back()
            .onTrue(runOnce({ coralStatus = CoralStatus.NONE; algaeStatus = AlgaeStatus.NONE }))

//        driveController.pov(90).onTrue(driveQueue.addButDoNotStartAsCommand({
//            PathfindingFactories.pathfindToCoralStation(
//                drive,
//                PathfindingFactories.CoralStationSide.RIGHT,
//                driveTranslationalControlSupplier
//            )
//        }))
//        driveController.pov(180).onTrue(driveQueue.addButDoNotStartAsCommand({
//            PathfindingFactories.pathfindToCoralStation(
//                drive, PathfindingFactories.CoralStationSide.LEFT, driveTranslationalControlSupplier
//            )
//        }))
        driveController.x().onTrue(runOnce({
            if (coralStatus == CoralStatus.NONE) coralStatus = CoralStatus.ON_INTAKE
        }))
        driveController.rightBumper()
            .onTrue(runOnce({ nextStation = PathfindingFactories.CoralStationSide.RIGHT }))
        driveController.leftBumper()
            .onTrue(runOnce({ nextStation = PathfindingFactories.CoralStationSide.LEFT }))
        driveController.y().onTrue(
            runOneFullCoralCycle()
        )
        // </editor-fold>


        //Trigger { abs(operatorController.leftY) > 0.1 }.whileTrue(
        //    arm.moveArm { -operatorController.leftY }
        //)
        //Trigger { abs(operatorController.rightY) > 0.1 }.whileTrue(
        //    elevator.velocityCommand { -operatorController.rightY }
        //)

        // Operator controller bindings
        operatorController.a().whileTrue(algaeIntake.runIntake())
        operatorController.b().onTrue(algaeIntake.retractIntake())
        operatorController.rightTrigger().whileTrue(
            elevator.velocityCommand { -operatorController.rightY }.alongWith(
                arm.moveArm { -operatorController.leftY },
            ),
        )
        operatorController.leftTrigger().whileTrue(
            climber.moveClimberToCageCatchPositionNoStop()
        )
        operatorController.x().onTrue(
            climber.climb()
        )

        operatorController.leftBumper()
            .whileTrue(clawIntake.spinFlywheel(Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 1.5))
        operatorController.rightBumper()
            .whileTrue(clawIntake.spinFlywheel(-Constants.SuperstructureConstants.ClawIntakeConstants.clawIntakePercent * 2.0))

        operatorController.start().whileTrue(climber.moveClimberOpenLoop({ -0.5 }, { 0.0 }))
        operatorController.back().whileTrue(climber.moveClimberOpenLoop({ 0.5 }, { 0.0 }))

        // Mark IV controller bindings


        hmi.b().onTrue(runOnce({
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L2
        }))
        hmi.y().onTrue(runOnce({
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L3
        }))
        hmi.x().onTrue(runOnce({
            nextPosition = Constants.SuperstructureConstants.SuperstructureState.L4
        }))

        hmi.povUp().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.povUpRight().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.povDownRight().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.povDown().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.povDownLeft().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.povUpLeft().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.leftBumper().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        hmi.rightBumper().onTrue(runOnce({
            updateHmi()
        }).ignoringDisable(true))
        //Trigger { drive.velocity > 2.0 && elevator.currentCommand == elevator.defaultCommand }.onTrue(
        //Trigger { drive.velocity > 2.0 && elevator.currentCommand == elevator.defaultCommand }.onTrue(
        //    SuperstructureCommands.home(elevator, arm)
        //)

        Trigger {
            DriverStation.getMatchTime() <= 21.0 && Robot.inTeleop && DriverStation.isFMSAttached()
        }.onTrue(
            runOnce(
                {
                    Elastic.selectTab("DROPCICK (Endgame)")
                })
        )

    }

    private fun runOneFullCoralCycle(): SequentialCommandGroup {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return goToCoralStation()

            // wait until the driver signals a coral on the intake (we have no way of detecting this)
            .andThen(
                lockWheelsAndWaitForInput()
            )
            // pathfind to reef; requires drive, elevator, arm, intake, climber
            .andThen(
                Commands.parallel(

                    // this command sequence concerns the drivebase + pathfinding
                    Commands.sequence(
                        // pathfinding speed; doesn't require anything
                        runOnce({
                            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                            inPosition = false
                        }),


                        // actually go to the reef
                        pathfindToReef().alongWith(
                            // wait until the claw has the coral secured;
                            // if we move too fast,
                            // we risk throwing it off the intake,
                            // but we still want to be able to move quickly
                            Commands.waitUntil { coralStatus == CoralStatus.IN_CLAW }.andThen({
                                // set pathfinding speed to the normal speed
                                drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                            })
                        ),

                        // at this point we're in the reef protected zone
                        // which should provide reprieve
                        // if the opposing alliance is playing defense
                        // and knocks us out of alignment.
                        // more importantly, we're stopped and waiting

                        // wait until the superstructure is in the position
                        // we want it to be in for scoring
                        Commands.waitUntil { inPosition }
                            // while we wait, lock the wheels
                            // this might actually help
                            // mitigate some of the "getting knocked out of alignment"
                            // issues we could face in a real match
                            .deadlineFor(Commands.run({ drive.stopWithX() }, drive)).andThen(
                                // close that last bit of distance
                                finalReefLineup()
                            ).withName("Pathfind to Reef (final)")
                    ), // end drivebase sequence

                    // wait until the claw has the coral secured and set that state
                    Commands.waitUntil { clawIntake.grabbed }
                        .andThen({ coralStatus = CoralStatus.IN_CLAW }),

                    // superstructure stuff
                    Commands.sequence(
                        // pick up the coral from the intake
                        SuperstructureCommands.pickUpCoral(
                            elevator, arm, clawIntake, climber
                        )
                            // i couldn't imagine a situation
                            // in which the coral wouldn't be on the intake,
                            // because this command can already only start
                            // once the driver confirms it,
                            // but this check could be useful at some point or another
                            .onlyIf { coralStatus == CoralStatus.ON_INTAKE },

                        // might fix an issue we were having
                        // where the arm tries to go to some random position
                        runOnce({ arm.setGoalToCurrent() }),

                        // wait until we're near where we need to be
                        Commands.waitUntil(drive::nearGoal).andThen(
                            // defer this command construction until it's called.
                            // that way,
                            // we're checking what position
                            // the drivers want the superstructure
                            // to be in at the last possible moment
                            // in case they change something.
                            // the field is dynamic, after all
                            Commands.defer({
                                // go to the position
                                SuperstructureCommands.goToPositionWithoutSafety(
                                    elevator, arm, climber, nextPosition
                                )
                            }, setOf(elevator, arm, climber))
                        ),
                        // once we're in position,
                        // set the state so the drivebase can continue its final lineup
                        runOnce({ inPosition = true })
                    )
                ) // end pathfinding to reef and lining up and all that jazz

            ).andThen(
                // deferred command, for the same reasons as the other one
                Commands.defer({
                    // score the coral
                    SuperstructureCommands.scoreAtPositionFaster(
                        elevator, arm, clawIntake, drive, nextPosition
                    )
                }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                    // set state of the coral
                    Runnable { coralStatus = CoralStatus.NONE })
                    // only do this deferred command + compositions
                    // if we actually have the coral
                    .onlyIf { coralStatus == CoralStatus.IN_CLAW })
    }

    private fun runOneFullCoralCycleButWaitTime(): SequentialCommandGroup {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return goToCoralStation()

            // wait until the driver signals a coral on the intake (we have no way of detecting this)
            .andThen(
                lockWheelsAndWaitTime()
            )
            // pathfind to reef; requires drive, elevator, arm, intake, climber
            .andThen(
                Commands.parallel(

                    // this command sequence concerns the drivebase + pathfinding
                    Commands.sequence(
                        // pathfinding speed; doesn't require anything
                        runOnce({
                            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                            inPosition = false
                        }),


                        // actually go to the reef
                        finalReefLineup(),
                        Commands.run({ drive.stopWithX() }, drive).withTimeout(0.1)
                    ), // end drivebase sequence

                    // wait until the claw has the coral secured and set that state
                    Commands.waitUntil { clawIntake.grabbed }
                        .andThen({
                            coralStatus = CoralStatus.IN_CLAW; drive.setPathfindingSpeedPercent(
                            Constants.PathfindingConstants.toReefSpeed
                        )
                        }),

                    // superstructure stuff
                    Commands.sequence(
                        // pick up the coral from the intake
                        SuperstructureCommands.pickUpCoral(
                            elevator, arm, clawIntake, climber
                        )
                            // i couldn't imagine a situation
                            // in which the coral wouldn't be on the intake,
                            // because this command can already only start
                            // once the driver confirms it,
                            // but this check could be useful at some point or another
                            .onlyIf { coralStatus == CoralStatus.ON_INTAKE },

                        // might fix an issue we were having
                        // where the arm tries to go to some random position
                        runOnce({ arm.setGoalToCurrent() }),

                        // wait until we're near where we need to be
                        // defer this command construction until it's called.
                        // that way,
                        // we're checking what position
                        // the drivers want the superstructure
                        // to be in at the last possible moment
                        // in case they change something.
                        // the field is dynamic, after all
                        Commands.defer({
                            // go to the position
                            SuperstructureCommands.goToPositionWithoutSafety(
                                elevator, arm, climber, nextPosition
                            )
                        }, setOf(elevator, arm, climber)),
                        // once we're in position,
                        // set the state so the drivebase can continue its final lineup
                        runOnce({ inPosition = true })
                    )
                ) // end pathfinding to reef and lining up and all that jazz

            ).andThen(
                // deferred command, for the same reasons as the other one
                Commands.defer({
                    // score the coral
                    SuperstructureCommands.scoreAtPositionFaster(
                        elevator, arm, clawIntake, drive, nextPosition
                    )
                }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                    // set state of the coral
                    Runnable { coralStatus = CoralStatus.NONE })
                    // only do this deferred command + compositions
                    // if we actually have the coral
                    .onlyIf { coralStatus == CoralStatus.IN_CLAW })
    }

    private fun updateReef(reef: PathfindingFactories.Reef) {
        nextReef = reef;
    }

    fun updateHmiAlgae() {
        // [0, 0.05]
        // between 0 and 6
        val joystickValue = (hmi.getRawAxis(0) / 0.05 * 6).roundToInt()
        if (joystickValue == 0) {
            grabAlgaeToggle = false
        } else {
            grabAlgaeToggle = true
            nextAlgaePosition = when (joystickValue) {
                1 -> PathfindingFactories.Reef.AB_ALGAE
                2 -> PathfindingFactories.Reef.CD_ALGAE
                3 -> PathfindingFactories.Reef.EF_ALGAE
                4 -> PathfindingFactories.Reef.GH_ALGAE
                5 -> PathfindingFactories.Reef.IJ_ALGAE
                else -> PathfindingFactories.Reef.KL_ALGAE
            }
        }

        Logger.recordOutput("HMI/NextReef", nextReef.name)
    }

    private fun updateHmi() {
        if (hmi.povUp().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.G
                false -> PathfindingFactories.Reef.H
            }
        }
        if (hmi.povUpRight().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.E
                false -> PathfindingFactories.Reef.F
            }
        }
        if (hmi.povDownRight().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.C
                false -> PathfindingFactories.Reef.D
            }
        }
        if (hmi.povDown().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.A
                false -> PathfindingFactories.Reef.B
            }
        }
        if (hmi.povDownLeft().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.K
                false -> PathfindingFactories.Reef.L
            }
        }
        if (hmi.povUpLeft().asBoolean) {
            nextReef = when (hmi.leftBumper().asBoolean) {
                true -> PathfindingFactories.Reef.I
                false -> PathfindingFactories.Reef.J
            }
        }
    }

    private fun lockWheelsAndWaitForInput(): Command =
        Commands.run({ drive.stopWithX() }, drive).withDeadline(
            Commands.waitUntil { coralStatus == CoralStatus.ON_INTAKE }.withName("Lock Wheels")
        )

    private fun lockWheelsAndWaitTime(): Command =
        Commands.run({ drive.stopWithX() }, drive).withDeadline(
            Commands.waitSeconds(0.5).andThen({ coralStatus = CoralStatus.ON_INTAKE })
                .withName("Lock Wheels")
        )

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()

    fun displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return

        Logger.recordOutput(
            "FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose
        )
        Logger.recordOutput(
            "FieldSimulation/Coral", *SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        )
        Logger.recordOutput(
            "FieldSimulation/Algae", *SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        )
    }

    private fun setUpAutoChooser() {
        // Set up auto routines
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())

        // Set up SysId routines
        // <editor-fold desc="SysId Routines">
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive)
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive)
        )

        autoChooser.addOption(
            "Pathfinding Auto (Far Left, Left, All)",
            AutoBuilder.buildAuto("Start at Far Left and Score Coral Left But End Halfway Through")
                .andThen({
                    updateReef(PathfindingFactories.Reef.J); nextPosition =
                    Constants.SuperstructureConstants.SuperstructureState.L4; nextStation =
                    PathfindingFactories.CoralStationSide.LEFT
                })
                .andThen(
                    runOneFullCoralCycleButWaitTime().alongWith(
                        Commands.waitSeconds(0.75).andThen(
                            { updateReef(PathfindingFactories.Reef.K) })
                    )
                )
                .andThen({ nextReef = PathfindingFactories.Reef.L })
                .andThen(runOneFullCoralCycleButWaitTime())
        )

        autoChooser.addOption(
            "Pathfinding Auto (Far Left, Left, Home Field)",
            AutoBuilder.buildAuto("Start at Far Left and Score Coral Left But End Halfway Through")
                .andThen({
                    updateReef(PathfindingFactories.Reef.J); nextPosition =
                    Constants.SuperstructureConstants.SuperstructureState.L4; nextStation =
                    PathfindingFactories.CoralStationSide.LEFT
                })
                .andThen(
                    runOneFullCoralCycleButWaitTime().alongWith(
                        Commands.waitSeconds(0.75).andThen(
                            { updateReef(PathfindingFactories.Reef.A) })
                    )
                )
                .andThen({ nextReef = PathfindingFactories.Reef.B })
                .andThen(runOneFullCoralCycleButWaitTime())
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, All)",
            AutoBuilder.buildAuto("Start at Mid Left and Score Coral Left But End Halfway Through")
                .andThen({
                    updateReef(PathfindingFactories.Reef.J); nextPosition =
                    Constants.SuperstructureConstants.SuperstructureState.L4; nextStation =
                    PathfindingFactories.CoralStationSide.LEFT
                })
                .andThen(
                    runOneFullCoralCycleButWaitTime().alongWith(
                        Commands.waitSeconds(0.75).andThen(
                            { updateReef(PathfindingFactories.Reef.K) })
                    )
                )
                .andThen({ nextReef = PathfindingFactories.Reef.L })
                .andThen(runOneFullCoralCycleButWaitTime())
        )

        autoChooser.addOption(
            "Pathfinding Auto (Mid Left, Left, Home Field)",
            AutoBuilder.buildAuto("Start at Mid Left and Score Coral Left But End Halfway Through")
                .andThen({
                    updateReef(PathfindingFactories.Reef.J); nextPosition =
                    Constants.SuperstructureConstants.SuperstructureState.L4; nextStation =
                    PathfindingFactories.CoralStationSide.LEFT
                }).andThen(
                    runOneFullCoralCycleButWaitTime().alongWith(
                        Commands.waitSeconds(0.75)
                            .andThen({ updateReef(PathfindingFactories.Reef.A) })
                    )
                )
                .andThen({ nextReef = PathfindingFactories.Reef.B })
                .andThen(runOneFullCoralCycleButWaitTime())
        )
        // </editor-fold>

    }

    fun setUpDashboardCommands() {


//        SmartDashboard.putData(
//
//            driveQueue.clearAllAsCommand().withName("Clear Auto Queue").alongWith(
//                superstructureQueue.clearAllAsCommand().withName("Clear Superstructure Queue")
//            ).withName("\nCLEAR ALL QUEUES\n")
//        )
        SmartDashboard.putData(
            elevator.homeElevator().deadlineFor(arm.moveArmToAngleWithoutEnding(90.0))
                .withName("Home Elevator")
        )
//
//
//
//        SmartDashboard.putData(algaeIntake.runIntake().withName("Run Intake"))
//        SmartDashboard.putData(algaeIntake.retractIntake().withName("Retract Intake"))


        // add individual reef goto commands to queue
        //for (reef in PathfindingFactories.Reef.entries) {
        //    SmartDashboard.putData(
        //        driveQueue.addAsCommand({
        //            PathfindingFactories.pathfindToSpecificReef(drive, reef)
        //                .withName("Reef " + reef.name + " (Queued)")
        //        }).withName("Queue Reef " + reef.name)
        //    )
        //}
//        for (reef in PathfindingFactories.Reef.entries) {
//            when (reef) {
//                PathfindingFactories.Reef.AB_ALGAE, PathfindingFactories.Reef.CD_ALGAE, PathfindingFactories.Reef.EF_ALGAE, PathfindingFactories.Reef.GH_ALGAE, PathfindingFactories.Reef.IJ_ALGAE, PathfindingFactories.Reef.KL_ALGAE -> {
//                    SmartDashboard.putData(
//                        driveQueue.addButDoNotStartAsCommand({
//                            PathfindingFactories.pathfindToReef(
//                                drive, reef, driveTranslationalControlSupplier
//                            )
//                        }).andThen(
//                            superstructureQueue.addButDoNotStartAsCommand({
//                                SuperstructureCommands.goToPosition(
//                                    elevator, arm, climber, when (reef) {
//                                        PathfindingFactories.Reef.AB_ALGAE, PathfindingFactories.Reef.EF_ALGAE, PathfindingFactories.Reef.IJ_ALGAE -> Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE
//                                        else -> Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
//                                    }
//                                )
//                            }, {
//                                clawIntake.intakeWithoutStoppingForAlgae()
//                            }, {
//                                SuperstructureCommands.goToPosition(
//                                    elevator,
//                                    arm,
//                                    climber,
//                                    Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
//                                )
//                            })
//                        ).withName("\nQueue Reef " + reef.name + "\n").ignoringDisable(true)
//                    )
//                }
//
//                else -> SmartDashboard.putData(driveQueue.addButDoNotStartAsCommand({
//                    PathfindingFactories.pathfindToReef(
//                        drive, reef, driveTranslationalControlSupplier
//                    )
//                }).withName("\nQueue Reef " + reef.name + "\n").ignoringDisable(true))
//
//
//            }
//
//        }

        //for (reef in PathfindingFactories.Reef.entries) {
        //    SmartDashboard.putData(
        //        driveQueue.addAsCommand({
        //            PathfindingFactories.finalLineupToSpecificReef(drive, reef)
        //                .withName("Final Lineup to Reef " + reef.name + " (Queued)")
        //        }).withName("Queue Final Lineup to Reef " + reef.name)
        //    )
        //}
//        for (coralStation in PathfindingFactories.CoralStationSide.entries) {
//            SmartDashboard.putData(
//                driveQueue.addAsCommand({
//                    PathfindingFactories.pathfindToSpecificCoralStation(
//                        drive, coralStation
//                    ).withName("Drive to " + coralStation.name + " Coral Station (Queued)")
//                }).withName("Queue Drive to " + coralStation.name + " Coral Station")
//            )
//            SmartDashboard.putData(
//                driveQueue.addButDoNotStartAsCommand({
//                    PathfindingFactories.pathfindToCoralStation(
//                        drive, coralStation, driveTranslationalControlSupplier
//                    ).withName("Drive to $coralStation Coral Station (Queued, alternate)")
//                }).ignoringDisable(true).andThen(
//                    superstructureQueue.addButDoNotStartAsCommand({
//                        SuperstructureCommands.preCoralPickup(elevator, arm, climber)
//                            .withName("Superstructure Pre-Coral Pickup (queued, auto-added)")
//                    }).withName("Queue Superstructure Pre-Coral Pickup").ignoringDisable(true)
//                ).andThen(
//
//                    superstructureQueue.addButDoNotStartAsCommand({
//                        SuperstructureCommands.pickUpCoral(elevator, arm, clawIntake, climber)
//                            .withName("Superstructure Coral Pickup (queued, auto-added)")
//                    }).withName("Queue Superstructure Coral Pickup").ignoringDisable(true)
//                ).withName("\n\n$coralStation Coral Station\n\n")
//            )
//        }


//        for (position in Constants.SuperstructureConstants.SuperstructureState.entries) {
//            val commands: Command = when (position) {
//                Constants.SuperstructureConstants.SuperstructureState.L1, Constants.SuperstructureConstants.SuperstructureState.L2, Constants.SuperstructureConstants.SuperstructureState.L3, Constants.SuperstructureConstants.SuperstructureState.L4 -> superstructureQueue.addButDoNotStartAsCommand(
//                    {
//                        SuperstructureCommands.goToPosition(
//                            elevator, arm, climber, position
//                        ).withName("Superstructure to " + position.name + " Position (Queued)")
//                    },
//                    {
//                        SuperstructureCommands.scoreAtPosition(
//                            elevator, arm, clawIntake, drive, position
//                        ).withName("Score in $position (queued, auto-added)")
//                    })
//
//                Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE, Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE -> superstructureQueue.addButDoNotStartAsCommand(
//                    {
//                        SuperstructureCommands.goToPosition(elevator, arm, climber, position)
//                            .withName("Superstructure to $position Position (Queued)")
//                    },
//                    {
//                        clawIntake.intakeWithoutStoppingForAlgae()
//                            .withName("Intake Algae (Queued, auto-added")
//                    },
//                    {
//                        SuperstructureCommands.goToPosition(
//                            elevator,
//                            arm,
//                            climber,
//                            Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
//                        ).withName("Superstructure to Barge Launch Position (Queued, auto-added)")
//                    })
//
//                else -> superstructureQueue.addButDoNotStartAsCommand({
//                    SuperstructureCommands.goToPosition(
//                        elevator, arm, climber, position
//                    ).withName("Superstructure to " + position.name + " Position (Queued)")
//                })
//            }
//            SmartDashboard.putData(
//                commands.withName("Queue Superstructure $position Position").ignoringDisable(true)
//            )
//        }
//        SmartDashboard.putData(
//            superstructureQueue.addButDoNotStartAsCommand(
//                {
//                    SuperstructureCommands.pickUpCoral(
//                        elevator, arm, clawIntake, climber
//                    ).withName("Pick up coral (queued)")
//                }).withName("Queue pick up coral").ignoringDisable(true)
//        )
//        SmartDashboard.putData(
//            superstructureQueue.addButDoNotStartAsCommand(
//                {
//                    SuperstructureCommands.score(
//                        elevator, arm, clawIntake
//                    ).withName("Score (queued)")
//                }).withName("Queue score").ignoringDisable(true)
//        )

//        SmartDashboard.putData(
//            driveQueue.addButDoNotStartAsCommand(
//                {
//                    PathfindingFactories.pathfindToPosition(
//                        drive, Pose2d(
//                            8.23, 4.84, Rotation2d(Units.Degrees.of(173.55))
//                        ).let {
//                            if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
//                                    .get() == DriverStation.Alliance.Red
//                            ) FlippingUtil.flipFieldPose(it) else it
//                        }, driveTranslationalControlSupplier
//                    )
//                }).withName("Pathfind to Home Cage")
//        )

//        SmartDashboard.putData(
//            driveQueue.addButDoNotStartAsCommand(
//                {
//                    PathfindingFactories.pathfindToPosition(
//                        drive, Pose2d(
//                            6.92, 6.12, Rotation2d(Units.Degrees.of(0.0))
//                        ).let {
//                            if (DriverStation.getAlliance().isPresent && DriverStation.getAlliance()
//                                    .get() == DriverStation.Alliance.Red
//                            ) FlippingUtil.flipFieldPose(it) else it
//                        }, driveTranslationalControlSupplier
//                    )
//                }).withName("Pathfind to Barge for Algae")
//        )
//        if (Constants.currentMode == Constants.Mode.SIM) SmartDashboard.putData(
//            runOnce(
//                { driveQueue.start() }).withName("Execute (sim-exclusive)")
//        )

        SmartDashboard.putData(CommandScheduler.getInstance())
//        SmartDashboard.putData("Auto Queue", driveQueue)
//        SmartDashboard.putData("Other Command Queue", superstructureQueue)
//        for (reef in PathfindingFactories.Reef.entries) {
//            SmartDashboard.putData(
//                runOnce({ nextReef = reef }).withName("Select Reef $reef")
//            )
//        }
//        for (station in PathfindingFactories.CoralStationSide.entries) {
//            SmartDashboard.putData(
//                runOnce({ nextStation = station }).withName("Select Coral Station $station")
//            )
//        }
//        for (pos in setOf(
//            Constants.SuperstructureConstants.SuperstructureState.L2,
//            Constants.SuperstructureConstants.SuperstructureState.L3,
//            Constants.SuperstructureConstants.SuperstructureState.L4
//        )) {
//            SmartDashboard.putData(
//                runOnce({ nextPosition = pos }).withName("Select Position $pos")
//            )
//        }

//        SmartDashboard.putData(
//            drive.followRepulsorField(Pose2d(15.7, 4.0, Rotation2d(Units.Degrees.of(180.0))))
//                .alongWith(
//                    SuperstructureCommands.algaeIntakeWithoutArm(elevator, arm, climber)
//                ).andThen(
//                    clawIntake.outtake().alongWith(
//                        Commands.run(
//                            { drive.stopWithX() }, drive
//                        )
//                    ).withDeadline(Commands.waitSeconds(5.0))
//                ).withName("Reset (RED ONLY)")
//        )
    }

    private fun finalReefLineup(): WrapperCommand = Commands.defer({
        PathfindingFactories.pathfindToReef(
            drive, { nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive)).withName("Pathfind to Reef")

    private fun pathfindToReef(): Command = Commands.defer({
        PathfindingFactories.pathfindToReefButBackALittle(
            drive, { nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive))

    private fun goToCoralStation(): Command =
        runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toCoralStationSpeed) }).andThen(
            Commands.defer(
                {
                    PathfindingFactories.pathfindToCoralStation(
                        drive, { nextStation }, driveTranslationalControlSupplier
                    )
                }, setOf(drive)
            ).deadlineFor(
                SuperstructureCommands.preCoralPickup(elevator, arm, climber)
            )
        )
            .finallyDo(Runnable { arm.setGoalToCurrent(); drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed) })
            .onlyIf { coralStatus == CoralStatus.NONE }
            .withName("Pathfind to Coral Station")

    private fun algaeCycle(): Command {

        return Commands.sequence(
            // back up, in case we're at the reef
            drive.backUp(),

            Commands.defer({
                runOnce({
                    // check if we have anything selected for barge positions.
                    // if not, default to none
                    bargePosition = bargeChooser.get() ?: BargePositions.NONE
                }).andThen(
                    // pathfind to where we need to be,
                    // just 20 inches back so we have room to swing the arm around
                    PathfindingFactories.pathfindToReefButBackALittleMore(
                        drive, { nextAlgaePosition }, driveTranslationalControlSupplier
                    )
                )
            }, setOf(drive)).alongWith(
                // wait until we're near the target reef,
                // then put the superstructure in whatever position we need to be in to grab algae
                Commands.waitUntil(drive::nearGoal).andThen(Commands.defer({
                    SuperstructureCommands.goToPosition(
                        elevator, arm, climber, when (nextAlgaePosition) {
                            PathfindingFactories.Reef.AB_ALGAE, PathfindingFactories.Reef.EF_ALGAE, PathfindingFactories.Reef.IJ_ALGAE -> Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE
                            else -> Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
                        }
                    )
                }, setOf(arm, elevator, climber)))
            ),

            // set the pathfinding speed
            // to be slower so we're not slamming into the reef at top speed
            runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.algaeGrabSpeed) }),

            // pathfind forward so we can actually pick the algae up
            PathfindingFactories.pathfindToReefButBackALittleLess(
                drive, { nextAlgaePosition }, driveTranslationalControlSupplier
            ).deadlineFor(
                // pin this onto the end
                // maybe it'll save us a little time?
                clawIntake.intakeWithoutStoppingForAlgae()
            ),

            // lock the wheels
            Commands.run(
                { drive.stopWithX() }, drive
            ).withDeadline(
                // spin the intake
                clawIntake.intakeWithoutStoppingForAlgae().withDeadline(
                    // wait until the motor stalls
                    // (surefire way to tell if we have an algae) and then wait an extra half-second
                    // FIXME: check if this value can go lower
                    Commands.waitUntil(clawIntake::grabbed).andThen(Commands.waitSeconds(0.5))
                    // set algae state
                ).andThen(runOnce({ algaeStatus = AlgaeStatus.IN_CLAW }))
            ).andThen(
                // back up
                drive.backUp()
            ).onlyIf { algaeStatus == AlgaeStatus.NONE }, // only do this if we don't have algae
            // decide what to do based on the barge position selection
            Commands.select(
                mapOf(
                    BargePositions.NONE to spitOutAlgae(),
                    BargePositions.LEFT to putAlgaeInBarge(),
                    BargePositions.MIDDLE to putAlgaeInBarge(),
                    BargePositions.RIGHT to putAlgaeInBarge()
                )
            ) { bargePosition } // use the barge position as the key
        )
    }

    private fun spitOutAlgae(): Command {
        return Commands.sequence(
            // rotate 180°, but stop after half a second
            DriveCommands.joystickDriveAtAngle(
                drive,
                { 0.0 },
                { 0.0 },
                { drive.pose.rotation.rotateBy(Rotation2d(Math.PI)) })
                .withDeadline(Commands.waitSeconds(0.5)),
            // spin the intake to spit out algae
            // no way to determine when this is done so just do it for 3/4 of a second
            Commands.run({ drive.stopWithX() }, drive).alongWith(
                clawIntake.outtakeFaster()
            ).withDeadline(
                Commands.waitSeconds(0.75)
            ),
            // set algae status to none, stop intake
            runOnce({
                algaeStatus = AlgaeStatus.NONE
            }).alongWith(clawIntake.stopOnce())
        )
            .finallyDo(Runnable { drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed) })
    }

    private fun putAlgaeInBarge(): Command {
        return Commands.sequence(
            // set pathfinding speed to whatever we have set for going to the barge
            runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toBargeSpeed) }),

            // pathfind to the barge
            Commands.defer({
                PathfindingFactories.pathfindToPosition(
                    drive,
                    Constants.PathfindingConstants.getPosition(bargePosition),
                    driveTranslationalControlSupplier
                )
            }, setOf(drive)).alongWith(
                // wait until we're near the barge
                Commands.waitSeconds(0.5).andThen(Commands.waitUntil(drive::nearGoal)).andThen(
                    // and then put the superstructure in the right position
                    SuperstructureCommands.goToPosition(
                        elevator,
                        arm,
                        climber,
                        Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
                    )
                )
                // while this is happening, spin the intake so we can keep the algae in
            ).deadlineFor(clawIntake.intakeWithoutStoppingForAlgae()),

            // drive forward while spitting the algae out.
            // the drive command ends after half a second
            drive.goForward().deadlineFor(
                clawIntake.outtake()
            ).andThen(
                // stop the intake and set algae status to none
                runOnce({
                    algaeStatus = AlgaeStatus.NONE
                }).alongWith(clawIntake.stopOnce())
            ),

            // back up a little bit
            drive.backUp(),

            // stop the drive
            // while we put the superstructure back into the position it needs to be in
            SuperstructureCommands.preCoralPickup(elevator, arm, climber).deadlineFor(
                runOnce({ drive.stopWithX() }, drive)
            )
        )
            .finallyDo(Runnable { drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed) })
    }


    private fun addNamedCommands() {
        NamedCommands.registerCommand(
            "home", SuperstructureCommands.home(elevator, arm, climber)
        )
        NamedCommands.registerCommand(
            "L4", SuperstructureCommands.l4WithoutSafety(elevator, arm)
        )
        NamedCommands.registerCommand(
            "deposit", SuperstructureCommands.scoreAtPositionWithoutDrive(
                elevator, arm, clawIntake, Constants.SuperstructureConstants.SuperstructureState.L4
            )
        )
        NamedCommands.registerCommand(
            "Pre-Coral Pickup", arm.moveArmToAngle(130.0).alongWith(elevator.goToPosition(37.0))
        )
        NamedCommands.registerCommand(
            "Fix Pivot",
            elevator.goToPosition(40.0).deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }))
                .andThen(
                    SuperstructureCommands.algaeIntakeWithoutSafety(
                        elevator, arm, climber
                    )
                )
        )
        NamedCommands.registerCommand(
            "Fix Pivot and L4", climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }).withDeadline(
                SuperstructureCommands.l4WithoutSafety(
                    elevator, arm
                )
            ).andThen(
                climber.moveClimberToIntakePosition().withDeadline(Commands.waitSeconds(0.25))
            )
        )
        NamedCommands.registerCommand(
            "Pick Up and L4", SuperstructureCommands.pickUpCoral(
                elevator, arm, clawIntake, climber
            ).raceWith(Commands.waitSeconds(3.75)).andThen(
                SuperstructureCommands.l4WithoutSafety(
                    elevator, arm
                )
            )
        )
        NamedCommands.registerCommand(
            "Low Algae", SuperstructureCommands.goToPosition(
                elevator,
                arm,
                climber,
                Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
            ).alongWith(
                clawIntake.intakeWithoutStoppingForAlgae().withName("Run Intake (auto-added)")
            )
        )
        NamedCommands.registerCommand(
            "Spit Out", clawIntake.outtakeFaster()
        )
        NamedCommands.registerCommand(
            "Barge Algae", SuperstructureCommands.goToPosition(
                elevator,
                arm,
                climber,
                Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH
            )
        )
        NamedCommands.registerCommand(
            "Lock Wheels", Commands.run({ drive.stopWithX() }, drive)
        )
        NamedCommands.registerCommand(
            "Run Intake", clawIntake.intakeWithoutStoppingForAlgae()
        )
    }

    fun stopQueue() {
        driveQueue.clearAll()
        superstructureQueue.clearAll()
    }

    fun fixArm() {
        if (climber.position > 90.0) elevator.goToPosition(40.0)
            .alongWith(arm.moveArmToAngle(Constants.SuperstructureConstants.ArmConstants.ArmState.PRE_CORAL_PICKUP))
            .deadlineFor(climber.moveClimberOpenLoop({ 0.0 }, { 0.0 }))
            .andThen(climber.moveClimberToIntakePosition().withDeadline(Commands.waitSeconds(1.0)))
            .andThen(
                SuperstructureCommands.preCoralPickup(
                    elevator, arm, climber
                )
            ).schedule()
    }

    fun disableAuto() {
        coralStatus = CoralStatus.NONE
    }
}

enum class CoralStatus {
    NONE, ON_INTAKE, IN_CLAW
}

enum class AlgaeStatus {
    NONE, IN_CLAW
}

enum class BargePositions {
    LEFT, MIDDLE, RIGHT, NONE
}

// Algae format for HMI input goes AB reef first, counterclockwise
