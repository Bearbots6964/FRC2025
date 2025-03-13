package frc.robot.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Arm(ArmIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage -> runCharacterization(voltage.in(Units.Volts))), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (DriverStation.isDisabled()) {
      io.stopArm();
    }
  }

  public Command stop() {
    return run(() -> io.holdArm(io.getArmAngleRotations())).withName("Arm Stop");
  }

  private void runCharacterization(double output) {
    io.setArmOpenLoopVoltage(output);
  }

  public Command sysIdDynamic(Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command sysIdQuasistatic(Direction direction) {
    return sysId.quasistatic(direction);
  }

  // TODO: Command Factories?

  public Command moveArm(DoubleSupplier output) {
    return run(() -> io.setArmOpenLoop(output.getAsDouble() * 0.2)).withName("Move Arm");
  }

  public Command moveArmToAngle(Double angle) {
    return run(() -> io.setArmAngle(angle))
        .until(() -> Math.abs(inputs.armAxisAngle - angle) < 3.0)
        .withName("Move Arm to Angle");
  }

  public Command moveArmAngleDelta(Double delta) {
    return runOnce(() -> io.setAngleDelta(delta))
        .until(() -> io.getDistanceFromGoal() < 3.0)
        .withName("Move Arm Delta");
  }
}
