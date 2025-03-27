package frc.robot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.signals.*
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.PathConstraints
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.math.util.Units.inchesToMeters
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
    /**
     * Constants object for the superstructure (elevator, arm, claw intake).
     */
    object SuperstructureConstants {
        /**
         * Constants object for the elevator.
         * Currently, the elevator is composed of a Thrifty Bot elevator kit with two Kraken X60 motors.
         */
        object ElevatorConstants {
            /**
             * Position of the elevator
             * at which the arm cannot possibly get stuck on the elevator crossbeam or algae intake.
             * Rotations.
             */
            const val armClearancePosition = 48.2

            /**
             * CAN ID of the left elevator motor.
             */
            const val LEFT_MOTOR_CAN_ID = 3

            /**
             * CAN ID of the right elevator motor.
             */
            const val RIGHT_MOTOR_CAN_ID = 2

            const val elevatorTolerance = 5.0 // rotations
            const val rotationsPerInch = 3.75


            /**
             * Talon FX configuration for the left elevator motor.
             */
            @JvmStatic
            val leftMotorConfig: TalonFXConfiguration = TalonFXConfiguration().let {
                // The Kotlin `let` scope function is really useful here,
                // as it allows us
                // to configure the object over multiple lines
                // without having to use an initializer block.
                it.ClosedLoopGeneral.ContinuousWrap = false
                it.MotorOutput.NeutralMode = NeutralModeValue.Brake
                it.CurrentLimits.StatorCurrentLimit = 20.0
                it.CurrentLimits.SupplyCurrentLimit = 20.0

                it // Implicit return.
                // This is really the only downside to using this, as it can look random.
            }

            /**
             * Talon FX configuration for the right elevator motor.
             */
            @JvmStatic
            val rightMotorConfig: TalonFXConfiguration = TalonFXConfiguration().let {
                it.ClosedLoopGeneral.ContinuousWrap = false
                it.MotorOutput.NeutralMode = NeutralModeValue.Brake
                it.CurrentLimits.StatorCurrentLimit = 20.0
                it.CurrentLimits.SupplyCurrentLimit = 20.0
                it.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

                it.Slot0.kS = 0.087054
                it.Slot0.kV = 0.11971
                it.Slot0.kA = 0.0031455
                it.Slot0.kG = 0.10784
                it.Slot0.kP = 0.8 // sysid suggests 0.067039

                it.MotionMagic.MotionMagicCruiseVelocity = 200.0
                it.MotionMagic.MotionMagicAcceleration = 400.0
                it.MotionMagic.MotionMagicJerk = 1000.0

                it
            }

            /**
             * Elevator height states.
             */
            object ElevatorState {
                const val HOME = 17.2
                const val L1 = 5.0
                const val L2 = 39.4
                const val L3 = 104.2
                const val L4 = 66.45 // TODO: Find actual value
                const val PRE_CORAL_PICKUP = 37.0
                const val CORAL_PICKUP = 48.25
                const val BARGE_LAUNCH = 100.93
                const val ALGAE_INTAKE = 0.0
                const val UPPER_REEF_ALGAE = 40.8
                const val LOWER_REEF_ALGAE = 0.0
            }

            /**
             * Whether SysID profiling is enabled for the elevator. Should probably be false.
             */
            const val SYSID_PROFILING_ENABLED = false
        }

        /**
         * Constants object for the arm.
         * As of now,
         * this is driven by a Neo motor
         * hooked up to a Talon FXS with a REV Through Bore Encoder jury-rigged in.
         */
        object ArmConstants {
            /**
             * Arm position states.
             */
            object ArmState {
                const val HOME = 210.5
                const val L1 = 155.0
                const val L2 = -62.93
                const val L3 = -67.85
                const val L4 = 55.81
                const val PRE_CORAL_PICKUP = HOME
                const val CORAL_PICKUP = 218.125
                const val BARGE_LAUNCH = 77.0
                const val ALGAE_INTAKE = -64.77
                const val UPPER_REEF_ALGAE = 0.0
                const val LOWER_REEF_ALGAE = 0.0
            }


            /**
             * Safe angle for the arm to be at when the elevator needs to move up..
             */
            const val safeAngle = -50.0

            /**
             * CAN ID of the arm motor.
             */
            @JvmStatic
            val armAxisMotorID: Int = 6

            /**
             * Talon FXS configuration for the arm motor.
             */
            @JvmStatic
            val talonConfig: TalonFXSConfiguration = TalonFXSConfiguration().let {
                it.CurrentLimits.StatorCurrentLimit = 40.0
                it.CurrentLimits.StatorCurrentLimitEnable = true

                it.ExternalFeedback.ExternalFeedbackSensorSource =
                    ExternalFeedbackSensorSourceValue.PulseWidth
                it.ExternalFeedback.RotorToSensorRatio = 60.0
                it.ExternalFeedback.QuadratureEdgesPerRotation = 8192
                it.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 0.65
                it.ExternalFeedback.AbsoluteSensorOffset = 0.9366666667
                it.ExternalFeedback.SensorPhase = SensorPhaseValue.Aligned

                it.MotorOutput.NeutralMode = NeutralModeValue.Brake
                it.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

                it.Slot0.kP = 40.0
                it.Slot0.kS = 0.51518
                it.Slot0.kV = 7.35
                it.Slot0.kA = 0.61
                it.Slot0.kG = 0.75
                it.Slot0.GravityType = GravityTypeValue.Arm_Cosine

                it.MotionMagic.MotionMagicCruiseVelocity = 1.25
                it.MotionMagic.MotionMagicAcceleration = 1.5
                it.MotionMagic.MotionMagicJerk = 4.0

                it.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled
                it.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST

                it.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                it.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                    Units.Degrees.of(225.0).`in`(Units.Revolutions)
                it.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
                it.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                    Units.Degrees.of(-71.36).`in`(Units.Revolutions)

                it // funny kotlin return
            }
        }

        /**
         * Constants object for the claw intake.
         * Currently a Neo 550 hooked up to a Spark Max.
         */
        object ClawIntakeConstants {
            /**
             * CAN ID of the claw motor.
             */
            @JvmStatic
            val clawMotorID: Int = 7

            /**
             * Percent output of the claw motor when intaking.
             */
            @JvmStatic
            val clawIntakePercent = 0.25

            const val clawGrippedCurrent = 8.0

            /**
             * Motor controller configuration for the claw motor.
             */
            @JvmStatic
            val sparkConfig: SparkBaseConfig = SparkMaxConfig().let {
                it.idleMode(SparkBaseConfig.IdleMode.kBrake)
                it.smartCurrentLimit(20) // Set relatively low to protect motor against stalling,
                // which seems to happen relatively frequently.
                // It stalls anyway,
                // but the whistle can be safely ignored;
                // Neos have been shown
                // to survive a 220-second stall at 40 amps with no performance degradation nor motor damage.
                // A match is only 150 seconds long, so we are more than okay.

                // no implicit return is needed here because smartCurrentLimit returns itself
            }
        }

        /**
         * Enum representing the various positions the superstructure can be in.
         */
        enum class SuperstructureState {
            HOME {
                override fun toString(): String {
                    return "Home"
                }
            },
            L1 {
                override fun toString(): String {
                    return "Level 1"
                }
            },
            L2 {
                override fun toString(): String {
                    return "Level 2"
                }
            },
            L3 {
                override fun toString(): String {
                    return "Level 3"
                }
            },
            L4 {
                override fun toString(): String {
                    return "Level 4"
                }
            },
            PRE_CORAL_PICKUP {
                override fun toString(): String {
                    return "Pre Coral Pickup"
                }
            },
            CORAL_PICKUP {
                override fun toString(): String {
                    return "Coral Pickup"
                }
            },
            BARGE_LAUNCH {
                override fun toString(): String {
                    return "Barge Algae Launch"
                }
            },
            ALGAE_INTAKE {
                override fun toString(): String {
                    return "Front Algae Intake"
                }
            },
            UPPER_REEF_ALGAE {
                override fun toString(): String {
                    return "Upper Reef Algae"
                }
            },
            LOWER_REEF_ALGAE {
                override fun toString(): String {
                    return "Lower Reef Algae"
                }
            }
        }
    }

    /**
     * Constants object for the algae intake.
     * Currently consists of a Neo 550 for actuation of the intake,
     * a Neo Vortex for the actual intake wheels,
     * and two Neo 550's on the side to help bump the algae in.
     * (One is currently off of the robot right now,
     * as the motor was hitting and breaking the wire chain.)
     */
    object AlgaeIntakeConstants {
        /** CAN ID of the arm motor. */
        @JvmStatic
        val armMotorID: Int = 4

        /** CAN ID of the intake motor. */
        @JvmStatic
        val intakeMotorID: Int = 5

        /** CAN ID of the left flywheel motor. */
        @JvmStatic
        val leftFlywheelMotorID: Int = 9

        /** CAN ID of the right flywheel motor. */
        @JvmStatic
        val rightFlywheelMotorID: Int = 8

        /** Percent output of the flywheel motor when running. */
        @JvmStatic
        val flywheelRunningPercent: Double = 0.16

        /** Velocity of the intake motor. */
        @JvmStatic
        val intakeVelocity: Double = 2000.0

        /** Position of the arm when extended. */
        @JvmStatic
        val armExtendedPosition: Double = 17.0

        /** Position of the arm when retracted. */
        @JvmStatic
        val armRetractedPosition: Double = -3.9

        /** Configuration for the arm motor. */
        @JvmStatic
        val armConfig = SparkMaxConfig().let {
            it.idleMode(SparkBaseConfig.IdleMode.kBrake)
            it.smartCurrentLimit(20)
            it.closedLoop.p(0.04).d(1.0)
            it
        }

        /** Configuration for the intake motor. */
        @JvmStatic
        val intakeConfig = SparkMaxConfig().let {
            it.idleMode(SparkBaseConfig.IdleMode.kCoast)
            it.smartCurrentLimit(40)
            it.closedLoop.velocityFF(0.0002)
            it
        }

        /** Configuration for the left flywheel motor. */
        @JvmStatic
        val leftFlywheelConfig = SparkMaxConfig().let {
            it.idleMode(SparkBaseConfig.IdleMode.kCoast)
            it.smartCurrentLimit(20)
            it.inverted(true)
            it
        }

        /** Configuration for the right flywheel motor. */
        @JvmStatic
        val rightFlywheelConfig = SparkMaxConfig().let {
            it.idleMode(SparkBaseConfig.IdleMode.kCoast)
            it.smartCurrentLimit(20)
            it.inverted(false)
            it
        }
    }

    object VisionConstants {
        // AprilTag layout
        @JvmStatic
        var aprilTagLayout: AprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField) // defaultField uses the welded field perimeter

        /**
         * Cameras for Pi `10.69.64.12`.
         */
        var frontRightCameraName: String = "front_right"
        var backRightCameraName: String = "back_right"

        /**
         * Cameras for Pi `10.69.64.11`.
         */
        var frontLeftCameraName: String = "front_left"
        var backLeftCameraName: String = "back_left"

        // Robot to camera transforms
        // (Not used by Limelight, configure in web UI instead)
        var robotToFrontRightCamera: Transform3d = Transform3d(
            // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
            // (frame side length / 2) - (distance from frame perimeter to camera)
            Units.Inches.of((29.5 / 2) - 1.3125),
            Units.Inches.of(-((29.5 / 2) - 3.75)),
            Units.Inches.of(8.125),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(32.29)),
        )
        var robotToBackRightCamera: Transform3d = Transform3d(
            Units.Inches.of(-((29.5 / 2) - 1.8125)),
            Units.Inches.of(-((29.5 / 2) - 1.5)),
            Units.Inches.of(8.0),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(225.0)),
        )
        var robotToFrontLeftCamera: Transform3d = Transform3d(
            Units.Inches.of((29.5 / 2) - 1.125),
            Units.Inches.of((29.5 / 2) - 3.375),
            Units.Inches.of(8.25),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(-26.5)),
        )
        var robotToBackLeftCamera: Transform3d = Transform3d(
            Units.Inches.of(-((29.5 / 2) - 1.8125)),
            Units.Inches.of((29.5 / 2) - 1.5),
            Units.Inches.of(8.0),
            Rotation3d(Units.Degrees.of(0.0), Units.Degrees.of(-20.0), Units.Degrees.of(135.0)),
        )

        // Basic filtering thresholds
        var maxAmbiguity: Double = 0.15
        var maxZError: Double = 0.25

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        var linearStdDevBaseline: Double = 0.2 // Meters
        var angularStdDevBaseline: Double = 0.6 // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        @JvmStatic
        var cameraStdDevFactors: DoubleArray = doubleArrayOf(
            1.0, // Front right camera
            3.0, // Back right camera
            1.0, // Front left camera
            3.0  // Back left camera
        )

        // Multipliers to apply for MegaTag 2 observations
        var linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
        var angularStdDevMegatag2Factor: Double =
            Double.POSITIVE_INFINITY // No rotation data available
    }

    object PathfindingConstants {
        /**
         * Final distance from the coral station in meters.
         */
        const val finalDistanceFromCoralStationMeters = 0.3737 // 16.77 inches

        /**
         * Distance from the goal at which pathfinding should end.
         */
        const val pathfindingEndDistanceFromGoal = 0.3

        /**
         * Final distance from the reef in meters.
         */
        const val finalDistanceFromReefMeters = 0.4768342

        /**
         * Lateral distance from the reef in meters.
         */
        const val lateralDistanceFromReefMeters = 0.1686306

        /**
         * Pathfinding constraints for the robot.
         */
        @JvmStatic
        val pathfindingConstraints = PathConstraints(
            1.0, 1.0, degreesToRadians(540.0), degreesToRadians(720.0)
        )

        /**
         * Constraints for the final lineup of the robot.
         */
        @JvmStatic
        val finalLineupConstraints = PathConstraints(0.6, 1.0, 2 * Math.PI, 4 * Math.PI)
    }

    object ClimberConstants {
        /**
         * CAN ID of the winch motor.
         */
        @JvmStatic
        val winchMotorID = 12

        /**
         * CAN ID of the pivot motor.
         */
        @JvmStatic
        val pivotMotorID = 11

        /**
         * Position of the pivot motor when the cage can catch the climber.
         */
        @JvmStatic
        val pivotCageCatchPosition = -45.0

        /**
         * Position of the pivot motor when fully retracted and climbed.
         */
        @JvmStatic
        val pivotClimbedPosition = 52.0


        /**
         * Talon FXS configuration for the pivot motor.
         */
        val pivotMotorConfig: TalonFXSConfiguration = TalonFXSConfiguration().let {
            it.MotorOutput.NeutralMode = NeutralModeValue.Brake

            it.CurrentLimits.StatorCurrentLimit = 40.0
            it.CurrentLimits.StatorCurrentLimitEnable = true

            it.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST
            it.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled

            it.ExternalFeedback.ExternalFeedbackSensorSource =
                ExternalFeedbackSensorSourceValue.PulseWidth
            it.ExternalFeedback.RotorToSensorRatio = 45.0
            it.ExternalFeedback.QuadratureEdgesPerRotation = 8192
            it.ExternalFeedback.AbsoluteSensorDiscontinuityPoint = 0.5
            it.ExternalFeedback.AbsoluteSensorOffset = -0.47

            it.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            it.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degrees.of(60.0).`in`(Units.Rotations)
            it.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
            it.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(-45.0).`in`(Units.Rotations)

            it.MotionMagic.MotionMagicCruiseVelocity = 0.325
            it.MotionMagic.MotionMagicAcceleration = 12.0

            it.Slot0.kP = 20.0

            it
        }

        /**
         * Talon FX configuration for the winch motor.
         */
        val winchMotorConfig: TalonFXConfiguration = TalonFXConfiguration().let {
            it.MotorOutput.NeutralMode = NeutralModeValue.Brake
            it.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

            it.CurrentLimits.SupplyCurrentLimit = 20.0
            it.CurrentLimits.StatorCurrentLimit = 20.0

            it.MotionMagic.MotionMagicCruiseVelocity = 10.0
            it.MotionMagic.MotionMagicAcceleration = 20.0

            it.Slot0.kP = 0.5
            it.Slot0.kS = 0.2

            it
        }
    }
    // <editor-fold desc="Extra utility constants">
    /**
     * What AdvantageKit should do while in a simulation.
     */
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

    /**
     * Enum representing zones of the field.
     */
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
        private val reef = Polygon(
            listOf(
                reefPoint1, reefPoint2, reefPoint3, reefPoint4, reefPoint5, reefPoint6
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

        fun inReef(point: Translation2d): Boolean {
            return reef.contains(point)
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
    var FRAME_WIDTH_METERS: Double = inchesToMeters(29.5)

    @JvmField
    var FRAME_LENGTH_METERS: Double = inchesToMeters(29.5)

    @JvmField
    var BUMPER_THICKNESS_METERS: Double = inchesToMeters(3.5)

    object PhysicalProperties {
        object ProgrammingBase {
            @JvmStatic
            val mass: Mass = Units.Pounds.of(150.0)

            @JvmStatic
            val momentOfInteria: MomentOfInertia = Units.KilogramSquareMeters.of(7.661) // TODO

            @JvmStatic
            val wheelRadius: Distance = Units.Inches.of(1.91)

            // We are using Kraken X60's. 15.5 ft/s without field-oriented control,
            // and 15.0 ft/s without.
            @JvmStatic
            val maxVelocity: LinearVelocity = Units.FeetPerSecond.of(15.5)

            @JvmStatic
            val coefficentOfFriction = 1.2 // this will need to be changed.

            // just a ballpark estimate for now.
            // very much depends on whether we're on carpet or concrete.
            object Config {

                private val motor: DCMotor =
                    DCMotor.getKrakenX60(1).withReduction(6.75) // per module
                private val currentLimit: Current = Units.Amps.of(75.0)

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
    // </
}
