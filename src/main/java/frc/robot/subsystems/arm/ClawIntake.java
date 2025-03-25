package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ClawIntake extends SubsystemBase {

  private final ClawIntakeIO io;
  private final ClawIntakeIOInputsAutoLogged inputs = new ClawIntakeIOInputsAutoLogged();
  @Getter public boolean manuallySetToGrabbed = false;

  public ClawIntake(ClawIntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (DriverStation.isDisabled()) {
      io.stopIntake();
    }
  }

  public Command spinFlywheel(Double output) {
    return run(() -> io.setIntakeOpenLoop(output)).withName("Spin Intake");
  }

  public Command stop() {
    return run(io::stopIntake).withName("Intake Stop");
  }

  public Command stopOnce() {
    return runOnce(io::stopIntake).withName("Intake Stop");
  }

  public Command intake() {
    return run(() ->
            io.setIntakeOpenLoop(
                -SuperstructureConstants.ClawIntakeConstants.getClawIntakePercent()))
        .until(() -> inputs.thingGripped || manuallySetToGrabbed)
        .andThen(io::stopIntake)
        .finallyDo(io::stopIntake)
        .withName("Intake");
  }

  public Command outtake() {
    return run(() ->
            io.setIntakeOpenLoop(
                SuperstructureConstants.ClawIntakeConstants.getClawIntakePercent()))
        .finallyDo(() -> spinFlywheel(0.0))
        .withName("Outtake");
  }
  public Command outtakeFaster() {
    return run(() ->
                   io.setIntakeOpenLoop(
                       SuperstructureConstants.ClawIntakeConstants.getClawIntakePercent() + 0.2))
        .finallyDo(() -> spinFlywheel(0.0))
        .withName("Outtake");
  }

  public void setIntakeGrabbed(Boolean grabbed) {
    this.manuallySetToGrabbed = grabbed;
  }
}
