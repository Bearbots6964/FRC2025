package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClawIntake extends SubsystemBase {
  private final ClawIntakeIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public ClawIntake(ClawIntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ClawIntake", inputs);

    if (DriverStation.isDisabled()) {
      io.stopFlywheel();
    }
  }

  public Command spinFlywheel(Double output) {
    return run(() -> io.setFlywheelOpenLoop(output)).withName("Spin ClawIntake");
  }

  public Command intakeFlywheel() {
    return run(() -> io.setFlywheelOpenLoop(Constants.FlywheelConstants.getFlywheelIntakePercent()))
        .until(() -> inputs.limitSwitchPressed)
        .finallyDo(() -> spinFlywheel(0.0))
        .withName("Intake ClawIntake");
  }

  public Command stop() {
    return run(io::stopFlywheel).withName("ClawIntake Stop");
  }
}
