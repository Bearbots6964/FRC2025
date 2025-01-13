package frc.robot.generated

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import frc.robot.subsystems.CommandSwerveDrivetrain

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
object TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private val steerGains: Slot0Configs = Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0.0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private val driveGains: Slot0Configs = Slot0Configs()
        .withKP(0.1).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.124)

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private val kSteerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val kDriveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The type of motor used for the drive motor
    private val kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated

    // The type of motor used for the drive motor
    private val kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private val kSteerFeedbackType = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private val kSlipCurrent: Current = Units.Amps.of(40.0)

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private val driveInitialConfigs = TalonFXConfiguration()
    private val steerInitialConfigs: TalonFXConfiguration = TalonFXConfiguration()
        .withCurrentLimits(
            CurrentLimitsConfigs() // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Units.Amps.of(60.0))
                .withStatorCurrentLimitEnable(true)
        )
    private val encoderInitialConfigs = CANcoderConfiguration()

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private val pigeonConfigs: Pigeon2Configuration? = null

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    val kCANBus: CANBus = CANBus("Drivebase", "./logs/example.hoot")

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    val kSpeedAt12Volts: LinearVelocity = Units.FeetPerSecond.of(12.5)

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private const val kCoupleRatio = 3.5714285714285716

    private const val kDriveGearRatio = 6.746031746031747
    private const val kSteerGearRatio = 21.428571428571427
    private val kWheelRadius: Distance = Units.Inches.of(2.0)

    private const val kInvertLeftSide = false
    private const val kInvertRightSide = true

    private const val kPigeonId = 14

    // These are only used for simulation
    private val kSteerInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)
    private val kDriveInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)

    // Simulated voltage necessary to overcome friction
    private val kSteerFrictionVoltage: Voltage = Units.Volts.of(0.2)
    private val kDriveFrictionVoltage: Voltage = Units.Volts.of(0.2)

    val DrivetrainConstants: SwerveDrivetrainConstants = SwerveDrivetrainConstants()
        .withCANBusName(kCANBus.name)
        .withPigeon2Id(kPigeonId)
        .withPigeon2Configs(pigeonConfigs)

    private val ConstantCreator: SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)


    // Front Left
    private const val kFrontLeftDriveMotorId = 4
    private const val kFrontLeftSteerMotorId = 6
    private const val kFrontLeftEncoderId = 5
    private val kFrontLeftEncoderOffset: Angle = Units.Rotations.of(-0.354736328125)
    private const val kFrontLeftSteerMotorInverted = true
    private const val kFrontLeftEncoderInverted = false

    private val kFrontLeftXPos: Distance = Units.Inches.of(12.125)
    private val kFrontLeftYPos: Distance = Units.Inches.of(12.125)

    // Front Right
    private const val kFrontRightDriveMotorId = 1
    private const val kFrontRightSteerMotorId = 3
    private const val kFrontRightEncoderId = 2
    private val kFrontRightEncoderOffset: Angle = Units.Rotations.of(-0.321044921875)
    private const val kFrontRightSteerMotorInverted = true
    private const val kFrontRightEncoderInverted = false

    private val kFrontRightXPos: Distance = Units.Inches.of(12.125)
    private val kFrontRightYPos: Distance = Units.Inches.of(-12.125)

    // Back Left
    private const val kBackLeftDriveMotorId = 7
    private const val kBackLeftSteerMotorId = 9
    private const val kBackLeftEncoderId = 8
    private val kBackLeftEncoderOffset: Angle = Units.Rotations.of(0.011474609375)
    private const val kBackLeftSteerMotorInverted = true
    private const val kBackLeftEncoderInverted = false

    private val kBackLeftXPos: Distance = Units.Inches.of(-12.125)
    private val kBackLeftYPos: Distance = Units.Inches.of(12.125)

    // Back Right
    private const val kBackRightDriveMotorId = 10
    private const val kBackRightSteerMotorId = 12
    private const val kBackRightEncoderId = 11
    private val kBackRightEncoderOffset: Angle = Units.Rotations.of(0.31103515625)
    private const val kBackRightSteerMotorInverted = true
    private const val kBackRightEncoderInverted = false

    private val kBackRightXPos: Distance = Units.Inches.of(-12.125)
    private val kBackRightYPos: Distance = Units.Inches.of(-12.125)


    val FrontLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            kFrontLeftXPos,
            kFrontLeftYPos,
            kInvertLeftSide,
            kFrontLeftSteerMotorInverted,
            kFrontLeftEncoderInverted
        )
    val FrontRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            kFrontRightXPos,
            kFrontRightYPos,
            kInvertRightSide,
            kFrontRightSteerMotorInverted,
            kFrontRightEncoderInverted
        )
    val BackLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            kBackLeftXPos,
            kBackLeftYPos,
            kInvertLeftSide,
            kBackLeftSteerMotorInverted,
            kBackLeftEncoderInverted
        )
    val BackRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            kBackRightXPos,
            kBackRightYPos,
            kInvertRightSide,
            kBackRightSteerMotorInverted,
            kBackRightEncoderInverted
        )

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    fun createDrivetrain(): CommandSwerveDrivetrain {
        return CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        )
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    open class TunerSwerveDrivetrain : SwerveDrivetrain<TalonFX?, TalonFX?, CANcoder?> {
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
        ) : super(
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, *modules
        )

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
        ) : super(
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, odometryUpdateFrequency, *modules
        )

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
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<TalonFX?> { deviceId: Int, canbus: String? ->
                TalonFX(
                    deviceId,
                    canbus
                )
            },
            DeviceConstructor<CANcoder?> { deviceId: Int, canbus: String? ->
                CANcoder(
                    deviceId,
                    canbus
                )
            },
            drivetrainConstants, odometryUpdateFrequency,
            odometryStandardDeviation, visionStandardDeviation, *modules
        )
    }
}
