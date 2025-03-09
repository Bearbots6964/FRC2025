// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

/**
 * Originally by FRC team 254, modified by Michael Jansen, and further modified by 167. See the
 * license files in the root directory of this project.
 *
 * <p>Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;
  private final DCMotor driveMotor;
  private final double statorCurrentLimitAmps,
      maxDriveVelocity,
      massKg,
      moiKgMetersSquared,
      wheelRadiusMeters,
      wheelFrictionForce,
      maxTorqueFriction;
  private final double[] maxSteerSpeedRadPerSec;
  private final SimpleMatrix forceKinematics;

  public SwerveSetpointGenerator(
      final Translation2d[] moduleLocations,
      final DCMotor driveMotor,
      final double statorCurrentLimitAmps,
      final double[] maxSteerSpeedRadPerSec,
      final double massKg,
      final double moiKgMetersSquared,
      final double wheelDiameterMeters,
      final double wheelCoF) {

    this.driveMotor = driveMotor;
    this.statorCurrentLimitAmps = statorCurrentLimitAmps;
    this.maxSteerSpeedRadPerSec = maxSteerSpeedRadPerSec;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    this.moduleLocations = moduleLocations;
    this.massKg = massKg;
    this.moiKgMetersSquared = moiKgMetersSquared;
    this.wheelRadiusMeters = wheelDiameterMeters / 2;
    this.maxDriveVelocity = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;

    wheelFrictionForce = wheelCoF * ((massKg / 4) * 9.81);
    maxTorqueFriction = this.wheelFrictionForce * wheelRadiusMeters;

    forceKinematics = new SimpleMatrix(8, 3);
    for (int i = 0; i < 4; i++) {
      var modPosReciprocal =
          new Translation2d(
              1.0 / this.moduleLocations[i].getNorm(), this.moduleLocations[i].getAngle());
      forceKinematics.setRow(i * 2, 0, /* Start Data */ 1, 0, -modPosReciprocal.getY());
      forceKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, modPosReciprocal.getX());
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  private final double EPSILON = 1e-6;

  private boolean chassisSpeedsNearZero(ChassisSpeeds speeds) {
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, EPSILON)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, EPSILON)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, EPSILON);
  }

  private boolean epsilonEquals(double a, double b) {
    return MathUtil.isNear(a, b, EPSILON);
  }

  @FunctionalInterface
  private interface Function2d {
    double f(double x, double y);
  }

  public record SwerveSetpoint(
      ChassisSpeeds chassisSpeeds,
      SwerveModuleState[] moduleStates,
      double[] steerFeedforwardsRadPerSec,
      double[] driveFeedforwardsAmps)
      implements StructSerializable {
    public static Struct<SwerveSetpoint> struct =
        new Struct<>() {
          @Override
          public Class<SwerveSetpoint> getTypeClass() {
            return SwerveSetpoint.class;
          }

          @Override
          public String getTypeName() {
            return "SwerveSetpoint";
          }

          @Override
          public int getSize() {
            return ChassisSpeeds.struct.getSize()
                + (SwerveModuleState.struct.getSize() + kSizeDouble * 2) * 4;
          }

          @Override
          public String getSchema() {
            return "ChassisSpeeds chassisSpeeds;SwerveModuleState moduleStates[4];double "
                + "steerFeedforwardsRadPerSec[4];double driveFeedforwardsAmps[4]";
          }

          @Override
          public SwerveSetpoint unpack(ByteBuffer bb) {
            var chassisSpeeds = ChassisSpeeds.struct.unpack(bb);
            var moduleStates = Struct.unpackArray(bb, 4, SwerveModuleState.struct);
            var steerFeedforwards = Struct.unpackDoubleArray(bb, 4);
            var driveFeedforwards = Struct.unpackDoubleArray(bb, 4);
            return new SwerveSetpoint(
                chassisSpeeds, moduleStates, steerFeedforwards, driveFeedforwards);
          }

          @Override
          public void pack(ByteBuffer bb, SwerveSetpoint value) {
            ChassisSpeeds.struct.pack(bb, value.chassisSpeeds);
            Struct.packArray(bb, value.moduleStates, SwerveModuleState.struct);
            Struct.packArray(bb, value.steerFeedforwardsRadPerSec);
            Struct.packArray(bb, value.driveFeedforwardsAmps);
          }

          @Override
          public Struct<?>[] getNested() {
            return new Struct<?>[] {ChassisSpeeds.struct, SwerveModuleState.struct};
          }
        };
  }

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 8);
  }

  protected double findDriveMaxS(
      double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func = (x, y) -> Math.hypot(x, y) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 10);
  }

  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredState) {
    return generateSetpoint(
        prevSetpoint, desiredState, RobotController.getBatteryVoltage(), Robot.defaultPeriodSecs);
  }

  private enum ActiveConstraint {
    STEER_VEL,
    CENTRIPETAL,
    MOTOR_DYNAMICS,
    STATOR_CURRENT,
    TRACTION
  }

  private final SimpleMatrix chassisForceMatrix = new SimpleMatrix(3, 1);

  /**
   * Generate a new setpoint.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredChassisSpeeds The desired state of motion, such as from the driver sticks or a
   *     path following algorithm.
   * @param dt The loop time.
   * @return A setpoint that satisfies all the kinematic limits while converging to desiredState
   *     quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredChassisSpeeds,
      double voltage,
      double dt) {
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Last Setpoint",
        SwerveSetpoint.struct,
        prevSetpoint);
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Desired chassis speeds", desiredChassisSpeeds);

    if (voltage > 12) {
      voltage = 12;
    }
    double maxSpeed = maxDriveVelocity * Math.min(1, voltage / 12);

    SwerveModuleState[] desiredModuleStates;
    if (Math.hypot(desiredChassisSpeeds.vxMetersPerSecond, desiredChassisSpeeds.vyMetersPerSecond)
            < .005
        && Math.abs(desiredChassisSpeeds.omegaRadiansPerSecond) < .01) {
      desiredChassisSpeeds.vxMetersPerSecond = 0;
      desiredChassisSpeeds.vyMetersPerSecond = 0;
      desiredChassisSpeeds.omegaRadiansPerSecond = 0;
      desiredModuleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    } else {
      // Some crimes against RAM right here. This wouldn't be a big deal if the RoboRIO
      // wasn't crap.
      desiredModuleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
      // Make sure desiredState respects velocity limits.
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeed);
      desiredChassisSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);
      // Discretize
      desiredChassisSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, dt);
      desiredModuleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
      // Desaturate
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeed);
      desiredChassisSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);
    }

    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Desaturated chassis speeds",
        desiredChassisSpeeds);

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    if (chassisSpeedsNearZero(desiredChassisSpeeds)) {
      need_to_steer = false;
      for (int i = 0; i < 4; ++i) {
        desiredModuleStates[i].angle = prevSetpoint.moduleStates()[i].angle;
        desiredModuleStates[i].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[4];
    double[] prev_vy = new double[4];
    Rotation2d[] prev_heading = new Rotation2d[4];
    double[] desired_vx = new double[4];
    double[] desired_vy = new double[4];
    Rotation2d[] desired_heading = new Rotation2d[4];
    boolean all_modules_should_flip = true;
    for (int i = 0; i < 4; ++i) {
      prev_vx[i] =
          prevSetpoint.moduleStates()[i].angle.getCos()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.moduleStates()[i].angle.getSin()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
      if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      desired_vx[i] =
          desiredModuleStates[i].angle.getCos() * desiredModuleStates[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleStates[i].angle.getSin() * desiredModuleStates[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleStates[i].angle;
      if (desiredModuleStates[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.kPi);
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !chassisSpeedsNearZero(prevSetpoint.chassisSpeeds())
        && !chassisSpeedsNearZero(desiredChassisSpeeds)) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
      // of the desired
      // angle, and accelerate again.
      return generateSetpoint(prevSetpoint, new ChassisSpeeds(), voltage, dt);
    }

    // Compute the deltas between start and goal. We can then interpolate from the start state to
    // the goal state; then
    // find the amount we can move from start towards goal in this cycle such that no kinematic
    // limit is exceeded.
    double dx =
        desiredChassisSpeeds.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
    double dy =
        desiredChassisSpeeds.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredChassisSpeeds.omegaRadiansPerSecond
            - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

    ActiveConstraint[] activeConstraints = new ActiveConstraint[4];

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since
    // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(4);
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle,
    // and then backing out the maximum interpolant between start and goal states. We remember the
    // minimum across all modules, since
    // that is the active constraint.
    for (int i = 0; i < 4; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
        continue;
      }
      double max_theta_step = dt * maxSteerSpeedRadPerSec[i];
      overrideSteering.add(Optional.empty());
      if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based
        // purely on rotation in place.
        if (epsilonEquals(desiredModuleStates[i].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
          continue;
        }

        var necessaryRotation =
            prevSetpoint
                .moduleStates()[i]
                .angle
                .unaryMinus()
                .rotateBy(desiredModuleStates[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
        }
        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(i, Optional.of(desiredModuleStates[i].angle));
          // Don't limit the global min_s;
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.moduleStates()[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          activeConstraints[i] = ActiveConstraint.STEER_VEL;
          min_s = 0.0;
        }
        continue;
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      // Enforce centripetal force limits to prevent sliding.
      // We do this by changing max_theta_step to the maximum change in heading over dt
      // that would create a large enough radius to keep the centripetal force under the
      // friction force.
      double maxHeadingChange =
          (dt * wheelFrictionForce)
              / ((massKg / 4) * Math.abs(prevSetpoint.moduleStates()[i].speedMetersPerSecond));
      max_theta_step = Math.min(max_theta_step, maxHeadingChange);

      activeConstraints[i] =
          max_theta_step == maxHeadingChange
              ? ActiveConstraint.CENTRIPETAL
              : ActiveConstraint.STEER_VEL;

      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel torque limits
    Translation2d chassisForceVec = new Translation2d();
    double chassisTorque = 0.0;
    int[] forceSigns = new int[4];
    double[] desiredSpeeds = new double[4];
    double[] expectedCurrentDraws = new double[4];
    double[] lastVelMagnitudes = new double[4];
    Translation2d[] moduleForces = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      double prevSpeed = prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      desiredModuleStates[i].optimize(prevSetpoint.moduleStates()[i].angle);
      double desiredSpeed = desiredModuleStates[i].speedMetersPerSecond;

      desiredSpeeds[i] = desiredSpeed;

      int forceSign;
      Rotation2d forceAngle = prevSetpoint.moduleStates()[i].angle;
      if (epsilonEquals(prevSpeed, 0.0)
          || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
          || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
        forceSign = 1; // Force will be applied in direction of module
        if (prevSpeed < 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      } else {
        forceSign = -1; // Force will be applied in opposite direction of module
        if (prevSpeed > 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      }

      forceSigns[i] = forceSign;

      // Estimate duty cycle from stator current and duty cycle
      double lastVelRadPerSec =
          Math.abs(prevSetpoint.moduleStates()[i].speedMetersPerSecond / wheelRadiusMeters);
      lastVelMagnitudes[i] = lastVelRadPerSec;

      double currentDraw;
      if (epsilonEquals(prevSpeed, desiredSpeed)) {
        currentDraw = 0;
      } else if (forceSign == 1) {
        // Use the current battery voltage since we won't be able to supply
        // 12v if the battery is sagging down to 11v, which will affect the max torque output
        currentDraw =
            Math.max(
                Math.min(
                    driveMotor.getCurrent(Math.abs(lastVelRadPerSec), voltage),
                    statorCurrentLimitAmps),
                0);
      } else {
        currentDraw = statorCurrentLimitAmps;
      }
      double moduleTorque = driveMotor.getTorque(currentDraw);

      // Limit torque to prevent wheel slip
      moduleTorque = Math.min(moduleTorque, maxTorqueFriction);

      if (moduleTorque == maxTorqueFriction) {
        activeConstraints[i] = ActiveConstraint.TRACTION;
        currentDraw = driveMotor.getCurrent(maxTorqueFriction);
      } else if (currentDraw == statorCurrentLimitAmps) {
        activeConstraints[i] = ActiveConstraint.STATOR_CURRENT;
      } else if (!epsilonEquals(0, currentDraw)) {
        activeConstraints[i] = ActiveConstraint.MOTOR_DYNAMICS;
      }
      expectedCurrentDraws[i] = currentDraw;

      double forceAtCarpet = moduleTorque / wheelRadiusMeters;
      Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);

      moduleForces[i] = moduleForceVec;

      // Add the module force vector to the chassis force vector
      chassisForceVec = chassisForceVec.plus(moduleForceVec);

      // Calculate the torque this module will apply to the chassis
      if (!epsilonEquals(0, moduleForceVec.getNorm())) {
        Rotation2d angleToModule = moduleLocations[i].getAngle();
        Rotation2d theta = moduleForceVec.getAngle().minus(angleToModule);
        chassisTorque += forceAtCarpet * moduleLocations[i].getNorm() * theta.getSin();
      }
    }

    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Expected stator current/", expectedCurrentDraws);
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Force Signs", forceSigns);
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Desired Wheel Speeds", desiredSpeeds);
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Last Vel Mag", lastVelMagnitudes);

    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Module Force Vecs", moduleForces);
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Internal State/Chassis Force Vec", chassisForceVec);
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Chassis Torque", chassisTorque);

    Translation2d chassisAccelVec = chassisForceVec.div(massKg);
    double chassisAngularAccel = chassisTorque / moiKgMetersSquared;

    // Use kinematics to convert chassis accelerations to module accelerations
    var chassisAccel =
        new ChassisSpeeds(chassisAccelVec.getX(), chassisAccelVec.getY(), chassisAngularAccel);
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Chassis Accel", chassisAccel);
    var accelStates = kinematics.toSwerveModuleStates(chassisAccel);
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Accel States", accelStates);

    var maxVelSteps = new double[4];
    for (int i = 0; i < 4; i++) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }

      double maxVelStep = Math.max(Math.abs(accelStates[i].speedMetersPerSecond * dt), .01);
      maxVelSteps[i] = maxVelStep;

      double vx_min_s =
          min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster than that.
      double s =
          findDriveMaxS(
              prev_vx[i],
              prev_vy[i],
              Math.hypot(prev_vx[i], prev_vy[i]),
              vx_min_s,
              vy_min_s,
              Math.hypot(vx_min_s, vy_min_s),
              maxVelStep);
      min_s = Math.min(min_s, s);
    }
    Logger.recordOutput("Swerve/Setpoint Generator/Internal State/Max Vel Step", maxVelSteps);

    var retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.chassisSpeeds.vxMetersPerSecond + min_s * dx,
            prevSetpoint.chassisSpeeds.vyMetersPerSecond + min_s * dy,
            prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
    var retStates = kinematics.toSwerveModuleStates(retSpeeds);
    // Desaturate
    SwerveDriveKinematics.desaturateWheelSpeeds(retStates, maxSpeed);
    retSpeeds = kinematics.toChassisSpeeds(retStates);
    // Discretize
    retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);
    retStates = kinematics.toSwerveModuleStates(retSpeeds);
    // Desaturate
    SwerveDriveKinematics.desaturateWheelSpeeds(retStates, maxSpeed);
    retSpeeds = kinematics.toChassisSpeeds(retStates);

    double chassisAccelXMpss =
        (retSpeeds.vxMetersPerSecond - prevSetpoint.chassisSpeeds.vxMetersPerSecond) / dt;
    double chassisAccelYMpss =
        (retSpeeds.vyMetersPerSecond - prevSetpoint.chassisSpeeds.vyMetersPerSecond) / dt;
    double chassisForceXNewtons = chassisAccelXMpss * massKg;
    double chassisForceYNewtons = chassisAccelYMpss * massKg;

    double angularAccel =
        (retSpeeds.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond) / dt;
    double angularTorqueNewtonMeters = angularAccel * moiKgMetersSquared;
    chassisForceMatrix.setColumn(
        0, 0, chassisForceXNewtons / 4, chassisForceYNewtons / 4, angularTorqueNewtonMeters / 4);
    var forcesMatrix = forceKinematics.mult(chassisForceMatrix);

    var steerFeedforwards = new double[4];
    var driveFeedforwards = new double[4];
    for (int i = 0; i < 4; ++i) {
      var wheelForce =
          new Translation2d(forcesMatrix.get(i * 2, 0), forcesMatrix.get(i * 2 + 1, 0));
      var wheelForceMagnitude = wheelForce.getNorm();
      var appliedForce =
          wheelForceMagnitude > EPSILON
              ? wheelForceMagnitude * wheelForce.getAngle().minus(retStates[i].angle).getCos()
              : 0;
      var torqueCurrent =
          MathUtil.clamp(
              driveMotor.getCurrent(appliedForce * wheelRadiusMeters),
              -statorCurrentLimitAmps,
              statorCurrentLimitAmps);

      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
          torqueCurrent *= -1;
        }
        retStates[i].angle = override;
      }
      final var deltaRotation =
          prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.kPi);
        retStates[i].speedMetersPerSecond *= -1.0;
        torqueCurrent *= -1;
      }

      steerFeedforwards[i] =
          retStates[i].angle.minus(prevSetpoint.moduleStates()[i].angle).getRadians() / dt;
      driveFeedforwards[i] = torqueCurrent;

      Logger.recordOutput(
          "Swerve/Setpoint Generator/Internal State/Active constraints/" + i, activeConstraints[i]);
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(retStates, maxDriveVelocity);

    var setpoint =
        new SwerveSetpoint(
            kinematics.toChassisSpeeds(retStates), retStates, steerFeedforwards, driveFeedforwards);
    Logger.recordOutput("Swerve/Setpoint Generator/Setpoint", SwerveSetpoint.struct, setpoint);
    return setpoint;
  }
}
