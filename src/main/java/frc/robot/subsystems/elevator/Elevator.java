package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private SysIdRoutine sysIdRoutine;

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
    }
  }


  public void setRotations(double rotations) {
    io.setPosition(rotations);
  }

  public Command stop() {
    return run(io::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command velocityCommand(DoubleSupplier joystick) {
    return run(() -> io.setOpenLoop(joystick.getAsDouble()));
  }

  public Command sysIdQuasistatic(Direction direction) {
    if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
      return sysIdRoutine.quasistatic(direction);
    } else {
      return null;
    }
  }

  public Command sysIdDynamic(Direction direction) {
    if (ElevatorConstants.SYSID_PROFILING_ENABLED) {
      return sysIdRoutine.dynamic(direction);
    } else {
      return null;
    }
  }

  public Command goToPosition(ElevatorConstants.ElevatorState state) {
    var position = 0.0;
    switch (state) {
      case L1:
        position = ElevatorConstants.L1;
        break;
      case L2:
        position = ElevatorConstants.L2;
        break;
      case L3:
        position = ElevatorConstants.L3;
        break;
      case L4:
        position = ElevatorConstants.L4;
        break;
      case HOME:
      default:
        position = ElevatorConstants.HOME;
        break;
    }
    double finalPosition = position;
    return run(() -> setRotations(finalPosition));
  }

  public Command goToPosition(double position) {
    return run(() -> setRotations(position))
        .until(() -> Math.abs(inputs.rightMotorPositionRotations - position) < 1.0);
  }

  public Command doNothing() {
    return run(io::stop);
  }
}
