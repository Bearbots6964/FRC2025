package frc.robot.subsystems

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Constants
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain
import java.util.function.Supplier

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class CommandSwerveDrivetrain : TunerSwerveDrivetrain, Subsystem {
    private var simNotifier: Notifier? = null
    private var lastSimTime = 0.0

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false

    /** Swerve request to apply during robot-centric path following  */
    private val pathApplyRobotSpeeds = ApplyRobotSpeeds()

    /* Swerve requests to apply during SysId characterization */
    private val translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val sysIdRoutineTranslation = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(4.0),  // Reduce dynamic step voltage to 4 V to prevent brownout
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State ->
            SignalLogger.writeString(
                "SysIdTranslation_State", state.toString()
            )
        }, Mechanism(
            { output: Voltage? -> setControl(translationCharacterization.withVolts(output)) },
            null,
            this
        )
    )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private val sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(7.0),  // Use dynamic voltage of 7 V
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State ->
            SignalLogger.writeString(
                "SysIdSteer_State", state.toString()
            )
        }, Mechanism(
            { volts: Voltage? -> setControl(steerCharacterization.withVolts(volts)) }, null, this
        )
    )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private val sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Units.Volts.of(Math.PI / 6)
                .per(Units.Second),  /* This is in radians per second, but SysId only supports "volts" */
            Units.Volts.of(Math.PI), null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State ->
            SignalLogger.writeString(
                "SysIdRotation_State", state.toString()
            )
        }, Mechanism(
            { output: Voltage ->
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(rotationCharacterization.withRotationalRate(output.`in`(Units.Volts)))/* also log the requested output for SysId */
                SignalLogger.writeDouble(
                    "Rotational_Rate", output.`in`(Units.Volts)
                )
            }, null, this
        )
    )

    /* The SysId routine to test */
    private val sysIdRoutineToApply = sysIdRoutineTranslation

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    constructor(
        drivetrainConstants: SwerveDrivetrainConstants,
        vararg modules: SwerveModuleConstants<*, *, *>?
    ) : super(drivetrainConstants, *modules) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on
     * CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    constructor(
        drivetrainConstants: SwerveDrivetrainConstants,
        odometryUpdateFrequency: Double,
        vararg modules: SwerveModuleConstants<*, *, *>?
    ) : super(drivetrainConstants, odometryUpdateFrequency, *modules) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on
     * CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * in the form [x, y, theta]ᵀ, with units in meters
     * and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     * in the form [x, y, theta]ᵀ, with units in meters
     * and radians
     * @param modules                   Constants for each specific module
     */
    constructor(
        drivetrainConstants: SwerveDrivetrainConstants,
        odometryUpdateFrequency: Double,
        odometryStandardDeviation: Matrix<N3?, N1?>,
        visionStandardDeviation: Matrix<N3?, N1?>,
        vararg modules: SwerveModuleConstants<*, *, *>?
    ) : super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        *modules
    ) {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configureAutoBuilder()
    }

    /**
     * Method to configure PathPlanner settings.
     */
    private fun configureAutoBuilder() {
        try {
            val config = Constants.PhysicalProperties.activeBase.robotConfig
            AutoBuilder.configure(
                { state.Pose },  // Supplier of current robot pose
                { pose: Pose2d? -> this.resetPose(pose) },  // Consumer for seeding pose against auto
                { state.Speeds },  // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                { speeds: ChassisSpeeds?, feedforwards: DriveFeedforwards ->
                    setControl(
                        pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    )
                },
                PPHolonomicDriveController( // TODO tune
                    PIDConstants(10.0, 0.0, 0.0), // PID constants for translation
                    PIDConstants(7.0, 0.0, 0.0)  // PID constants for rotation
                ),
                config,
                {
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                },  // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                this, // Subsystem for requirements
            )
        } catch (ex: Exception) {
            DriverStation.reportError(
                "Failed to load PathPlanner config and configure AutoBuilder", ex.stackTrace
            )
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return sysIdRoutineToApply.quasistatic(direction)
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return sysIdRoutineToApply.dynamic(direction)
    }

    override fun periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red) kRedAlliancePerspectiveRotation
                    else kBlueAlliancePerspectiveRotation
                )
                hasAppliedOperatorPerspective = true
            }
        }
    }

    private fun startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - lastSimTime
            lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    companion object {
        private const val kSimLoopPeriod = 0.005 // 5 ms

        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero

        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        private val kRedAlliancePerspectiveRotation: Rotation2d = Rotation2d.k180deg
    }
}
