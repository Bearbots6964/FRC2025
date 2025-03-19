package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (DriverStation.isDisabled()) {
      io.stopFlywheel();
    }
  }
  public Command spinFlywheel(Double output) {
    return run(() -> io.setFlywheelOpenLoop(output)).withName("Spin Flywheel");
  }

  public Command intakeFlywheel() {
    return run(() -> io.setFlywheelOpenLoop(Constants.FlywheelConstants.getFlywheelIntakePercent()))
        .until(() -> inputs.limitSwitchPressed)
        .finallyDo(() -> spinFlywheel(0.0))
        .withName("Intake Flywheel");
  }
  public Command stop() {
    return run(io::stopFlywheel).withName("Flywheel Stop");
  }
}
