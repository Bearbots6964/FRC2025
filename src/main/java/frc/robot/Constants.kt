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
import frc.robot.util.Polygon

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    object OperatorConstants {
        const val DRIVER_FULL_AUTO = true
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


            val slot0Configs = rightMotorConfig.Slot0
            slot0Configs.kS = 0.12893
            slot0Configs.kV = 0.12063
            slot0Configs.kA = 0.0018313
            slot0Configs.kG = 0.092435
            slot0Configs.kP = 0.8

            val motionMagicConfigs = rightMotorConfig.MotionMagic
            motionMagicConfigs.MotionMagicCruiseVelocity = 100.0
            motionMagicConfigs.MotionMagicAcceleration = 200.0
            motionMagicConfigs.MotionMagicJerk = 1000.0
        }

        const val HOME = 5.0
        const val L1 = 5.0
        const val L2 = 13.5
        const val L3 = 50.0
        const val L4 = 102.3 // TODO: Find actual value
        const val CORAL_PICKUP = 40.0

        enum class ElevatorState {
            HOME, L1, L2, L3, L4, CORAL_PICKUP
        }

        const val SYSID_PROFILING_ENABLED = false
    }

    object PhysicalProperties {
        object ProgrammingBase {
            @JvmStatic
            val mass: Mass = Units.Pounds.of(125.0)

            @JvmStatic
            val momentOfInteria: MomentOfInertia = Units.KilogramSquareMeters.of(2.0) // TODO

            @JvmStatic
            val wheelRadius: Distance = Units.Inches.of(1.913)

            // We are using Kraken X60's. 15.5 ft/s without field-oriented control,
            // and 15.0 ft/s without.
            @JvmStatic
            val maxVelocity: LinearVelocity = Units.FeetPerSecond.of(15.5)

            @JvmStatic
            val coefficentOfFriction = 0.8 // this will need to be changed.

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

    object ArmConstants {
        class ArmState {
            companion object {
                const val HOME = 160.0
                const val L1 = 155.0
                const val L2 = 160.0
                const val L3 = 155.0
                const val L4 = 165.0
                const val CORAL_PICKUP = HOME
            }
        }


        @JvmStatic
        val armAxisMotorID: Int = 6
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
        var camera2Name: String = "back"

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
        var robotToCamera2: Transform3d = Transform3d(
            Units.Inches.of(-1.25),
            Units.Inches.of(0.0),
            Units.Inches.of(46.0),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-10.0), Units.Degrees.of(180.0)),
        )

        // Basic filtering thresholds
        var maxAmbiguity: Double = 0.2
        var maxZError: Double = 0.25

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        var linearStdDevBaseline: Double = 0.5 // Meters
        var angularStdDevBaseline: Double = 0.6 // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        @JvmStatic
        var cameraStdDevFactors: DoubleArray = doubleArrayOf(
            1.0,  // Camera 0
            1.0, // Camera 1
            3.0 // Camera 2
        )

        // Multipliers to apply for MegaTag 2 observations
        var linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
        var angularStdDevMegatag2Factor: Double =
            Double.POSITIVE_INFINITY // No rotation data available
    }


    private val simMode: Mode = Mode.SIM

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

    enum class Zone {
        LOWER_CORAL_STATION {
            override fun toString(): String {
                return "Lower Coral Station"
            }
        },
        UPPER_CORAL_STATION {
            override fun toString(): String {
                return "Upper Coral Station"
            }
        },
        REEF_AB {
            override fun toString(): String {
                return "Reef AB"
            }
        },
        REEF_CD {
            override fun toString(): String {
                return "Reef CD"
            }
        },
        REEF_EF {
            override fun toString(): String {
                return "Reef EF"
            }
        },
        REEF_GH {
            override fun toString(): String {
                return "Reef GH"
            }
        },
        REEF_IJ {
            override fun toString(): String {
                return "Reef IJ"
            }
        },
        REEF_KL {
            override fun toString(): String {
                return "Reef KL"
            }
        },
        NONE {
            override fun toString(): String {
                return "None"
            }
        }
    }

    object FieldConstants {
        // Field zoning.
        // Lower coral station
        private val lowerCoralStation = Polygon(
            listOf(
                // origin
                Translation2d(Units.Inches.of(0.0), Units.Inches.of(0.0)),
                // 124.406 in y
                Translation2d(Units.Inches.of(0.0), Units.Inches.of(124.406)),
                // 171.301 in x
                Translation2d(Units.Inches.of(171.301), Units.Inches.of(0.0)),
            ),
        ) // double-checked

        private val upperCoralStation = Polygon(
            listOf(
                // y = 192.594 in
                Translation2d(Units.Inches.of(0.0), Units.Inches.of(192.594)),
                // y = 317
                Translation2d(Units.Inches.of(0.0), Units.Inches.of(317.0)),
                // x = 171.301 in, y = 317 in
                Translation2d(Units.Inches.of(171.301), Units.Inches.of(317.0)),
            ),
        )

        // reef
        // the fms orders these starting at the top left and going clockwise
        // (2 letters per side, corresponding to the 2 poles per side)
        // reef is a hexagon, we can just extend it outward to get zones
        // first though let's get the points
        // starting at the top and going clockwise

        // 176.75, 196
        private val reefPoint1 = Translation2d(Units.Inches.of(176.75), Units.Inches.of(196.0))

        // 209.5, 177.5
        private val reefPoint2 = Translation2d(Units.Inches.of(209.5), Units.Inches.of(177.5))

        // 209.5, 139.5
        private val reefPoint3 = Translation2d(Units.Inches.of(209.5), Units.Inches.of(139.5))

        // 176.75, 120.5
        private val reefPoint4 = Translation2d(Units.Inches.of(176.75), Units.Inches.of(120.5))

        // 144, 139.5
        private val reefPoint5 = Translation2d(Units.Inches.of(144.0), Units.Inches.of(139.5))

        // 144, 177.5
        private val reefPoint6 = Translation2d(Units.Inches.of(144.0), Units.Inches.of(177.5))

        // now the bounds for the reef areas. these are the reef points extended outward 72 inches

        // 176.75, 279.5
        private val outerReefPoint1 = Translation2d(Units.Inches.of(176.75), Units.Inches.of(279.5))

        // 281.5, 219
        private val outerReefPoint2 = Translation2d(Units.Inches.of(281.5), Units.Inches.of(219.0))

        // 281.5, 98
        private val outerReefPoint3 = Translation2d(Units.Inches.of(281.5), Units.Inches.of(98.0))

        // 176.75, 37.5
        private val outerReefPoint4 = Translation2d(Units.Inches.of(176.75), Units.Inches.of(37.5))

        // 72, 98
        private val outerReefPoint5 = Translation2d(Units.Inches.of(72.0), Units.Inches.of(98.0))

        // 72, 219
        private val outerReefPoint6 = Translation2d(Units.Inches.of(72.0), Units.Inches.of(219.0))

        // now we create polygons for the reef zones. as previously stated, we're beginning the reef zones at the top left and going clockwise
        private val reefAreaAB = Polygon(
            listOf(
                reefPoint6, outerReefPoint6, outerReefPoint1, reefPoint1
            )
        )
        private val reefAreaCD = Polygon(
            listOf(
                reefPoint1, outerReefPoint1, outerReefPoint2, reefPoint2
            )
        )
        private val reefAreaEF = Polygon(
            listOf(
                reefPoint2, outerReefPoint2, outerReefPoint3, reefPoint3
            )
        )
        private val reefAreaGH = Polygon(
            listOf(
                reefPoint3, outerReefPoint3, outerReefPoint4, reefPoint4
            )
        )
        private val reefAreaIJ = Polygon(
            listOf(
                reefPoint4, outerReefPoint4, outerReefPoint5, reefPoint5
            )
        )
        private val reefAreaKL = Polygon(
            listOf(
                reefPoint5, outerReefPoint5, outerReefPoint6, reefPoint6
            )
        )

        fun getZone(point: Translation2d): Zone {
            if (lowerCoralStation.contains(point)) {
                return Zone.LOWER_CORAL_STATION
            } else if (upperCoralStation.contains(point)) {
                return Zone.UPPER_CORAL_STATION
            } else if (reefAreaAB.contains(point)) {
                return Zone.REEF_AB
            } else if (reefAreaCD.contains(point)) {
                return Zone.REEF_CD
            } else if (reefAreaEF.contains(point)) {
                return Zone.REEF_EF
            } else if (reefAreaGH.contains(point)) {
                return Zone.REEF_GH
            } else if (reefAreaIJ.contains(point)) {
                return Zone.REEF_IJ
            } else if (reefAreaKL.contains(point)) {
                return Zone.REEF_KL
            } else {
                return Zone.NONE
            }

        }


    }


    // compatibility
    // don't touch these
    @JvmField
    var FIELD_LENGTH_METERS = frc.robot.subsystems.FieldConstants.fieldLength

    @JvmField
    var FIELD_WIDTH_METERS = frc.robot.subsystems.FieldConstants.fieldWidth

    @JvmField
    var FIELD_CENTER: Translation2d = Translation2d(
        FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2
    )

    @JvmField
    var FRAME_WIDTH_METERS: Double = edu.wpi.first.math.util.Units.inchesToMeters(29.5)

    @JvmField
    var FRAME_LENGTH_METERS: Double = edu.wpi.first.math.util.Units.inchesToMeters(29.5)

    @JvmField
    var BUMPER_THICKNESS_METERS: Double = edu.wpi.first.math.util.Units.inchesToMeters(3.5)
}
