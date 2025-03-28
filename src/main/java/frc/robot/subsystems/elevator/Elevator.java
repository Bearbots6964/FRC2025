package frc.robot.subsystems.elevator;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SuperstructureConstants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants.SuperstructureState;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private SysIdRoutine sysIdRoutine;

  private double targetPosition;

  @Getter private MechanismLigament2d mechanism;

  public Elevator(ElevatorIO io) {
    this.io = io;
    if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
      SignalLogger.setPath("/U/sysidlogs/");
      SignalLogger.start();
      sysIdRoutine =
          new SysIdRoutine(
              new Config(
                  null,
                  Volts.of(4),
                  null,
                  (state) -> SignalLogger.writeString("state", state.toString())),
              new Mechanism((v) -> io.setVoltage(v.in(Volts)), null, this));

      targetPosition = inputs.rightMotorPositionRotations;
    }
    mechanism =
        new MechanismLigament2d(
            "Elevator", inchesToMeters(37.0), 90, inchesToMeters(2.0), new Color8Bit(Color.kGray));
  }

  public void setRotations(double rotations) {
    io.setPosition(rotations);
  }

  public Command stop() {
    return runOnce(io::setGoalToCurrent).andThen(run(io::stop)).withName("Elevator Stop");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    mechanism.setLength(inchesToMeters(inputs.rightMotorPositionRotations / 3.64 + 37));
  }

  /**
   * Command to move the elevator up or down based on a joystick input.
   *
   * @param joystick The joystick input to control the elevator
   * @return Command to move the elevator up or down based on a joystick input
   */
  public Command velocityCommand(DoubleSupplier joystick) {
    return run(() -> io.setOpenLoop(joystick.getAsDouble())).withName("Elevator Velocity Command");
  }

  /**
   * Run a quasistatic sysid routine on the elevator.
   *
   * @param direction The direction to run the sysid routine
   * @return The sysid routine command
   */
  public Command sysIdQuasistatic(Direction direction) {
    if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
      return sysIdRoutine.quasistatic(direction);
    } else {
      return null;
    }
  }

  /**
   * Run a dynamic sysid routine on the elevator.
   *
   * @param direction The direction to run the sysid routine
   * @return The sysid routine command
   */
  public Command sysIdDynamic(Direction direction) {
    if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
      return sysIdRoutine.dynamic(direction);
    } else {
      return null;
    }
  }

  /**
   * Command to go to a specific position. Accurate within one rotation (~1.187 inches).
   *
   * @param state The position to go to
   * @return Command to go to a specific position
   */
  public Command goToPosition(SuperstructureState state) {
    targetPosition =
        switch (state) {
          case L1 -> ElevatorConstants.ElevatorState.L1;
          case L2 -> ElevatorConstants.ElevatorState.L2;
          case L3 -> ElevatorConstants.ElevatorState.L3;
          case L4 -> ElevatorConstants.ElevatorState.L4;
          default -> ElevatorConstants.ElevatorState.HOME;
        };
    return run(() ->
            setRotations(
                switch (state) {
                  case L1 -> ElevatorConstants.ElevatorState.L1;
                  case L2 -> ElevatorConstants.ElevatorState.L2;
                  case L3 -> ElevatorConstants.ElevatorState.L3;
                  case L4 -> ElevatorConstants.ElevatorState.L4;
                  default -> ElevatorConstants.ElevatorState.HOME;
                }))
        .until(
            () ->
                Math.abs(
                        inputs.rightMotorPositionRotations
                            - switch (state) {
                              case L1 -> ElevatorConstants.ElevatorState.L1;
                              case L2 -> ElevatorConstants.ElevatorState.L2;
                              case L3 -> ElevatorConstants.ElevatorState.L3;
                              case L4 -> ElevatorConstants.ElevatorState.L4;
                              default -> ElevatorConstants.ElevatorState.HOME;
                            })
                    < ElevatorConstants.elevatorTolerance);
  }

  /**
   * Command to move the elevator up or down a specific number of rotations. Accurate within one
   * rotation (~1.187 inches).
   *
   * @param delta The number of rotations to move the elevator
   * @return Command to move the elevator up or down a specific number of rotations
   */
  public Command goToPositionDelta(double delta) {
    return runOnce(() -> io.setPositionDelta(delta))
        .until(() -> io.getDistanceFromGoal() < ElevatorConstants.elevatorTolerance)
        .withName("Elevator to Position");
  }

  /**
   * Command to go to a specific position. Accurate within one rotation (~1.187 inches).
   *
   * @param position The position to go to
   * @return Command to go to a specific position
   */
  public Command goToPosition(double position) {
    targetPosition = position;
    return run(() -> setRotations(position))
        .until(
            () ->
                Math.abs(inputs.rightMotorPositionRotations - position)
                    < ElevatorConstants.elevatorTolerance)
        .withName("Elevator to Position");
  }

  /**
   * Command to stop the elevator.
   *
   * @return Command to stop the elevator
   */
  public Command doNothing() {
    return run(io::stop).withName("Elevator Stop");
  }

  /**
   * Home the elevator. Uses current limit checking to determine when the elevator has reached the
   * bottom in lieu of a limit switch. This looks pretty cool. Reminds me of a 3D printer's homing
   * sequence.
   *
   * @return Command to home the elevator
   */
  public Command homeElevator() {
    return Commands.sequence(
        runOnce(() -> io.setSoftLimitsEnabled(false)),
        velocityCommand(() -> -0.25).until(() -> inputs.limitSwitchPressed),
        runOnce(io::zero),
        run(() -> setRotations(5))
            .until(
                () ->
                    Math.abs(inputs.rightMotorPositionRotations - 5)
                        < ElevatorConstants.elevatorTolerance),
        velocityCommand(() -> -0.05).until(() -> inputs.limitSwitchPressed),
        runOnce(io::zero),
        runOnce(() -> io.setOpenLoop(0.0)),
        runOnce(() -> io.setSoftLimitsEnabled(true)));
  }

  public void lockPosition() {
    targetPosition = inputs.rightMotorPositionRotations;
  }

  public double getPosition() {
    return inputs.rightMotorPositionRotations;
  }
}
