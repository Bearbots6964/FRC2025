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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static java.lang.Math.PI;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PhysicalProperties;
import frc.robot.Constants.PhysicalProperties.ProgrammingBase;
import frc.robot.Constants.Zone;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RepulsorFieldPlanner;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {

  public static final double DRIVE_BASE_RADIUS = Math.max(Math.max(
      Math.hypot(TunerConstants.getFrontLeft().LocationX, TunerConstants.getFrontLeft().LocationY),
      Math.hypot(TunerConstants.getFrontRight().LocationX,
                 TunerConstants.getFrontRight().LocationY)), Math.max(
      Math.hypot(TunerConstants.getBackLeft().LocationX, TunerConstants.getBackLeft().LocationY),
      Math.hypot(TunerConstants.getBackRight().LocationX,
                 TunerConstants.getBackRight().LocationY)));
  public static final DriveTrainSimulationConfig mapleSimConfig;
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.getDrivetrainConstants().CANBusName).isNetworkFD() ? 250.0 : 100.0;
  static final Lock odometryLock = new ReentrantLock();
  private static final RobotConfig PP_CONFIG = PhysicalProperties.getActiveBase().getRobotConfig();

  static {
    mapleSimConfig = DriveTrainSimulationConfig.Default().withRobotMass(ProgrammingBase.getMass())
        .withCustomModuleTranslations(getModuleTranslations()).withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1),
                                             TunerConstants.getFrontLeft().DriveMotorGearRatio,
                                             TunerConstants.getFrontLeft().SteerMotorGearRatio,
                                             Volts.of(
                                                 TunerConstants.getFrontLeft().DriveFrictionVoltage),
                                             Volts.of(
                                                 TunerConstants.getFrontLeft().SteerFrictionVoltage),
                                             Meters.of(TunerConstants.getFrontLeft().WheelRadius),
                                             KilogramSquareMeters.of(
                                                 TunerConstants.getFrontLeft().SteerInertia),
                                             ProgrammingBase.getCoefficentOfFriction()));
  }

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert(
      "Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      getModuleTranslations());
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
          new SwerveModulePosition(), new SwerveModulePosition()};
  private final Consumer<Pose2d> resetSimulationPoseCallBack;
  private final PIDController xController, yController, yawController;
  private final double maxLinearSpeedMetersPerSec = TunerConstants.getSpeedAt12Volts()
      .in(MetersPerSecond);
  private final SwerveSetpointGenerator setpointGenerator;
  private final RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwervePoseEstimator poseEstimator = new SwervePoseEstimator(kinematics,
                                                                            rawGyroRotation,
                                                                            lastModulePositions,
                                                                            new Pose2d());
  @Getter
  private Zone currentZone = Zone.NONE;
  private com.pathplanner.lib.util.swerve.SwerveSetpoint lastSetpoint;

  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO,
      ModuleIO brModuleIO, Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.getFrontLeft());
    modules[1] = new Module(frModuleIO, 1, TunerConstants.getFrontRight());
    modules[2] = new Module(blModuleIO, 2, TunerConstants.getBackLeft());
    modules[3] = new Module(brModuleIO, 3, TunerConstants.getBackRight());

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(this::getPose, this::setPose, this::getChassisSpeeds, this::runVelocity,
                          new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
                                                         new PIDConstants(5.0, 0.0, 0.0)),
                          PP_CONFIG,
                          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                          this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> Logger.recordOutput("Odometry/Trajectory",
                                            activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    // Configure SysId
    sysId = new SysIdRoutine(new SysIdRoutine.Config(null, null, null,
                                                     (state) -> Logger.recordOutput(
                                                         "Drive/SysIdState", state.toString())),
                             new SysIdRoutine.Mechanism(
                                 (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    Notifier zoneNotifier = new Notifier(
        () -> currentZone = FieldConstants.INSTANCE.getZone(getPose().getTranslation()));
    zoneNotifier.startPeriodic(0.5); // this really doesn't need to execute that often.
    // it's also a pretty resource-intensive operation, so we have to be careful with it

    xController = new PIDController(7.5, 0, 0);
    yController = new PIDController(7.5, 0, 0);
    yawController = new PIDController(4, 0, .2);
    yawController.enableContinuousInput(-PI, PI);

    setpointGenerator = new SwerveSetpointGenerator(PP_CONFIG, Units.rotationsToRadians(10.0));
    lastSetpoint = new com.pathplanner.lib.util.swerve.SwerveSetpoint(new ChassisSpeeds(),
                                                                      getModuleStates(),
                                                                      DriveFeedforwards.zeros(4));
  }

  /**
   * Returns an array of module translations.
   */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[]{new Translation2d(TunerConstants.getFrontLeft().LocationX,
                                                 TunerConstants.getFrontLeft().LocationY),
        new Translation2d(TunerConstants.getFrontRight().LocationX,
                          TunerConstants.getFrontRight().LocationY),
        new Translation2d(TunerConstants.getBackLeft().LocationX,
                          TunerConstants.getBackLeft().LocationY),
        new Translation2d(TunerConstants.getBackRight().LocationX,
                          TunerConstants.getBackRight().LocationY)};
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    // Drive standard deviations as a result of wheel slippage
    var driveStdDevs = getDriveStdDevs();
    Logger.recordOutput("Swerve/Odometry/Drive Std Devs", driveStdDevs);
    poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getCurrentMode() != Mode.SIM);

    Logger.recordOutput("Odometry/Zone", currentZone);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    currentSpeeds = speeds;
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.getSpeedAt12Volts());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive in a straight line with the specified drive output.
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /**
   * Stops the drive.
   */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * Returns a command to run a dynamic test in the specified direction.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all the modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all the modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Returns the measured chassis speeds of the robot.
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the position of each module in radians.
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * Returns the current odometry pose.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current odometry pose.
   */
  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Returns the current odometry rotation.
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Adds a new timestamped vision measurement.
   */
  @Override
  public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
                                       visionMeasurementStdDevs.getData());
  }

  /**
   * Returns the maximum linear speed in meters per sec.
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
  }

  /**
   * Returns the maximum angular speed in radians per sec.
   */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /**
   * Generates a command to pathfind and then follow the given path.
   *
   * @param path The path to follow.
   * @return A command that pathfinds and then follows the path.
   */
  public Command pathfindThenFollowPath(PathPlannerPath path) {
    var constraints = new PathConstraints(4.0, 4.0, 3 * Math.PI, 3.5 * Math.PI);
    return AutoBuilder.pathfindThenFollowPath(path, constraints)
        .withName("Pathfind then Follow Path");
  }

  /**
   * Returns a command that will back the robot up. Uses pathfinding with a pose transform.
   * Conservative translation constraints. Zero rotation constraints (no rotation allowed).
   *
   * @return a Command that will back the robot up
   */
  public Command backUpBy() {
    return run(() -> runVelocity(new ChassisSpeeds(-0.25, 0, 0))).raceWith(
        Commands.waitSeconds(1.0)).withName("Back Up");
  }

  // thank you to team 167 for the following code

  /**
   * Estimate drive-wheel slippage by comparing the actual wheel velocities to the idealized wheel
   * velocities. If there is a significant deviation, then a wheel(s) is slipping, and we should
   * raise the estimated standard deviation of the drivebase odometry to trust the wheel encoders
   * less.
   *
   * @return An array of length 3, containing the estimated standard deviations in each axis (x, y,
   * yaw)
   */
  private double[] getDriveStdDevs() {
    // Get idealized states from the current robot velocity.
    var idealStates = kinematics.toSwerveModuleStates(currentSpeeds);

    double xSquaredSum = 0;
    double ySquaredSum = 0;
    for (int i = 0; i < 4; i++) {
      var measuredVector = new Translation2d(getModuleStates()[i].speedMetersPerSecond,
                                             getModuleStates()[i].angle);
      var idealVector = new Translation2d(idealStates[i].speedMetersPerSecond,
                                          idealStates[i].angle);

      // Compare the state vectors and get the delta between them.
      var xDelta = idealVector.getX() - measuredVector.getX();
      var yDelta = idealVector.getY() - measuredVector.getY();

      // Square the delta and add it to a sum
      xSquaredSum += xDelta * xDelta;
      ySquaredSum += yDelta * yDelta;
    }

    // Sqrt of avg of squared deltas = standard deviation
    // Rotate to convert to field relative
    double scalar = 15;
    var stdDevs = new Translation2d(scalar * (Math.sqrt(xSquaredSum) / 4),
                                    scalar * (Math.sqrt(ySquaredSum) / 4)).rotateBy(
        gyroInputs.yawPosition);

    // If translating and rotating at the same time, odometry drifts pretty badly in the
    // direction perpendicular to the direction of translational travel.
    // This factor massively distrusts odometry in that direction when translating and rotating
    // at the same time.
    var scaledSpeed = new Translation2d(ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds,
                                                                              gyroInputs.yawPosition).vxMetersPerSecond
                                            / TunerConstants.getSpeedAt12Volts()
        .in(MetersPerSecond), ChassisSpeeds.fromFieldRelativeSpeeds(currentSpeeds,
                                                                    gyroInputs.yawPosition).vyMetersPerSecond
                                            / TunerConstants.getSpeedAt12Volts()
        .in(MetersPerSecond)).rotateBy(Rotation2d.kCCW_90deg).times(1 * Math.abs(
        currentSpeeds.omegaRadiansPerSecond / (
            TunerConstants.getSpeedAt12Volts().in(MetersPerSecond) / DRIVE_BASE_RADIUS)));

    // Add a minimum to account for mechanical slop and to prevent divide by 0 errors
    return new double[]{Math.abs(stdDevs.getX()) + Math.abs(scaledSpeed.getX()) + .1,
        Math.abs(stdDevs.getY()) + Math.abs(scaledSpeed.getY()) + .1, .001};
  }

  /**
   * Follows a repulsor field to a goal. The repulsor field is generated by the repulsorFieldPlanner
   * and is used to avoid obstacles.
   *
   * @param goal          Goal pose to navigate to
   * @param nudgeSupplier Supplier for a nudge vector to apply to the robot's velocity
   * @return Command to follow the repulsor field
   */
  public Command followRepulsorField(Pose2d goal, Supplier<Translation2d> nudgeSupplier) {
    return sequence(
        // reset the repulsor field planner and controllers, set the goal
        runOnce(() -> {
          repulsorFieldPlanner.setGoal(goal.getTranslation());
          xController.reset();
          yController.reset();
          yawController.reset();
        }), run(() -> {
          // log the goal pose
          Logger.recordOutput("Repulsor/Goal", goal);

          // get the repulsor field sample
          var sample = repulsorFieldPlanner.sampleField(
              poseEstimator.getEstimatedPosition().getTranslation(),
              maxLinearSpeedMetersPerSec * .9, 1.25);

          // calculate feedforward and feedback
          var feedforward = new ChassisSpeeds(sample.vx(), sample.vy(), 0);
          var feedback = new ChassisSpeeds(
              xController.calculate(poseEstimator.getEstimatedPosition().getX(),
                                    sample.intermediateGoal().getX()),
              yController.calculate(poseEstimator.getEstimatedPosition().getY(),
                                    sample.intermediateGoal().getY()), yawController.calculate(
              poseEstimator.getEstimatedPosition().getRotation().getRadians(),
              goal.getRotation().getRadians()));

          // log the error, feedforward, and feedback
          var error = goal.minus(poseEstimator.getEstimatedPosition());
          Logger.recordOutput("Repulsor/Error", error);
          Logger.recordOutput("Repulsor/Feedforward", feedforward);
          Logger.recordOutput("Repulsor/Feedback", feedback);

          // log the repulsor field
          // FIXME if this doesn't work, delete it
          Logger.recordOutput("Repulsor/Vector field", repulsorFieldPlanner.getArrows());

          // calculate the output field relative and robot relative speeds
          var outputFieldRelative = feedforward.plus(feedback);

          if (nudgeSupplier != null) {
            var nudge = nudgeSupplier.get();
            if (nudge.getNorm() > .1) {
              var nudgeScalar = Math.min(error.getTranslation().getNorm() / 3, 1) * Math.min(
                  error.getTranslation().getNorm() / 3, 1) * maxLinearSpeedMetersPerSec;

              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red) {
                nudge = new Translation2d(-nudge.getX(), -nudge.getY());
              }
              nudgeScalar *= Math.abs(nudge.getAngle().minus(
                  new Rotation2d(outputFieldRelative.vxMetersPerSecond,
                                 outputFieldRelative.vyMetersPerSecond)).getSin());
              outputFieldRelative.vxMetersPerSecond += nudge.getX() * nudgeScalar;
              outputFieldRelative.vyMetersPerSecond += nudge.getY() * nudgeScalar;
            }
          }

          var outputRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(outputFieldRelative,
                                                                          poseEstimator.getEstimatedPosition()
                                                                              .getRotation());

          var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, outputRobotRelative,
                                                            0.02);
          runVelocity(setpoint.robotRelativeSpeeds());
          lastSetpoint = setpoint;
        })).withName("Repulsor Field");
  }

  /**
   * Follows a repulsor field to a goal. The repulsor field is generated by the repulsorFieldPlanner
   * @param goal Goal pose to navigate to
   * @return Command to follow the repulsor field
   */
  public Command followRepulsorField(Pose2d goal) {
    return followRepulsorField(goal, null);
  }

  public double getVelocity() {
    return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }
}
