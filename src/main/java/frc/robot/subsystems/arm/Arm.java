package frc.robot.subsystems.arm;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
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
  private final Alert armAxisDisconnectedAlert;
  private final Alert armFlywheelDisconnectedAlert;
  private final SysIdRoutine sysId;

  public Arm(ArmIO io) {
    this.io = io;

    armAxisDisconnectedAlert = new Alert("Disconnected arm axis motor", Alert.AlertType.kError);
    armFlywheelDisconnectedAlert =
        new Alert("Disconnected arm flywheel motor", Alert.AlertType.kError);

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

  public void setArmFlywheelOpenLoop(double output) {
    io.setArmOpenLoop(output);
  }

  public void setArmAxisAngleDegrees(Angle angle) {
    io.setArmAngleDegrees(angle);
  }

  public Angle getArmAxisAngleDegrees() {
    return io.getArmAngleDegrees();
  }

  private void runCharacterization(double output) {
    io.setArmOpenLoop(output);
  }

  // TODO: SysID routines
  public Command sysIdDynamic(Direction direction) {
    return run(() -> sysId.dynamic(direction));
  }

  public Command sysIdQuasistatic(Direction direction) {
    return run(() -> sysId.quasistatic(direction));
  }

  // TODO: Command Factories?

  public Command moveArm(DoubleSupplier output) {
    return run(() -> io.setArmOpenLoop(output.getAsDouble()));
  }
}
