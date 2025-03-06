package frc.robot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.RobotBase

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
    }

    object ElevatorConstants {
        const val LEFT_MOTOR_CAN_ID = 3
        const val RIGHT_MOTOR_CAN_ID = 2

        @JvmStatic
        val leftMotorConfig: TalonFXConfiguration = TalonFXConfiguration()

        @JvmStatic
        val rightMotorConfig: TalonFXConfiguration = TalonFXConfiguration()

        init {
            leftMotorConfig.ClosedLoopGeneral.ContinuousWrap = false
            leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
            leftMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0
            leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0

            rightMotorConfig.ClosedLoopGeneral.ContinuousWrap = false
            rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
            rightMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0
            rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0


            var slot0Configs = rightMotorConfig.Slot0
            slot0Configs.kS = 0.12893
            slot0Configs.kV = 0.12063
            slot0Configs.kA = 0.0018313
            slot0Configs.kG = 0.092435
            slot0Configs.kP = 0.8

            var motionMagicConfigs = rightMotorConfig.MotionMagic
            motionMagicConfigs.MotionMagicCruiseVelocity = 100.0
            motionMagicConfigs.MotionMagicAcceleration = 200.0
            motionMagicConfigs.MotionMagicJerk = 1000.0
        }

        const val HOME = 0.0
        const val L1 = 40.0
        const val L2 = 50.0
        const val L3 = 85.0
        const val L4 = 150.0 // TODO: Find actual value

        enum class ElevatorState {
            HOME, L1, L2, L3, L4
        }

        const val SYSID_PROFILING_ENABLED = false
    }

    object PhysicalProperties {
        object ProgrammingBase {
            @JvmStatic
            val mass: Mass = Units.Pounds.of(60.0)

            @JvmStatic
            val momentOfInteria: MomentOfInertia = Units.KilogramSquareMeters.of(2.0) // TODO

            @JvmStatic
            val wheelRadius: Distance = Units.Inches.of(2.0)

            // We are using Kraken X60's. 15.5 ft/s without field-oriented control,
            // and 15.0 ft/s without.
            @JvmStatic
            val maxVelocity: LinearVelocity = Units.FeetPerSecond.of(15.5)

            @JvmStatic
            val coefficentOfFriction = 1.1 // this will need to be changed.

            // just a ballpark estimate for now.
            // very much depends on whether we're on carpet or concrete.
            object Config {

                private val motor: DCMotor =
                    DCMotor.getKrakenX60(1).withReduction(6.75) // per module
                private val currentLimit: Current = Units.Amps.of(40.0)

                private const val motorsPerModule = 1

                val config = ModuleConfig(
                    wheelRadius,
                    maxVelocity,
                    coefficentOfFriction,
                    motor,
                    currentLimit,
                    motorsPerModule
                )
            }

            private val moduleOffsets = arrayOf(
                // front left
                Translation2d(Units.Inches.of(12.125), Units.Inches.of(12.125)),
                // front right
                Translation2d(Units.Inches.of(12.125), Units.Inches.of(-12.125)),
                // back left
                Translation2d(Units.Inches.of(-12.125), Units.Inches.of(12.125)),
                // back right
                Translation2d(Units.Inches.of(-12.125), Units.Inches.of(-12.125)),
            )

            // construct the robot configuration.
            // note the asterisk before the moduleOffsets array -
            // that is the spread operator, allowing us to unwrap the array as a vararg
            val robotConfig = RobotConfig(mass, momentOfInteria, Config.config, *moduleOffsets)
        }

        @JvmStatic
        val activeBase = ProgrammingBase
    }

    object VisionConstants {
        // AprilTag layout
        @JvmStatic
        var aprilTagLayout: AprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

        /**
         * Camera for Pi `10.69.64.11`.
         */
        var camera0Name: String = "right"
        /**
         * Camera for Pi `10.69.64.12`.
         */
        var camera1Name: String = "left"

        // Robot to camera transforms
        // (Not used by Limelight, configure in web UI instead)
        var robotToCamera0: Transform3d = Transform3d(
            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            // (frame side length / 2) - (distance from frame perimeter to camera)
            Units.Inches.of((29.5 / 2) - 1.75),
            Units.Inches.of(-((29.5 / 2) - 3.5)),
            Units.Inches.of(8.25),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(30.0)),
        )
        var robotToCamera1: Transform3d = Transform3d(
            Units.Inches.of((29.5 / 2) - 1.75),
            Units.Inches.of((29.5 / 2) - 3.5),
            Units.Inches.of(8.25),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(-30.0)),
        )

        // Basic filtering thresholds
        var maxAmbiguity: Double = 0.3
        var maxZError: Double = 0.75

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        var linearStdDevBaseline: Double = 0.2 // Meters
        var angularStdDevBaseline: Double = 0.6 // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        @JvmStatic
        var cameraStdDevFactors: DoubleArray = doubleArrayOf(
            1.0,  // Camera 0
            1.0 // Camera 1
        )

        // Multipliers to apply for MegaTag 2 observations
        var linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
        var angularStdDevMegatag2Factor: Double =
            Double.POSITIVE_INFINITY // No rotation data available
    }


    val simMode: Mode = Mode.SIM

    @JvmStatic
    val currentMode: Mode = if (RobotBase.isReal()) Mode.REAL else simMode

    enum class Mode {
        /** Running on a real robot.  */
        REAL,

        /** Running a physics simulator.  */
        SIM,

        /** Replaying from a log file.  */
        REPLAY
    }
}
