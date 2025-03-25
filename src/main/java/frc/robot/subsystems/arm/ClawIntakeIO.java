package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIntakeIO {
  @AutoLog
  public class ClawIntakeIOInputs {
    public double intakeAppliedVoltage = 0.0;
    public double intakeAppliedCurrent = 0.0;
    public boolean limitSwitchPressed = false;
    public boolean thingGripped = false;
  }

  default void updateInputs(ClawIntakeIOInputs inputs) {}

  default void setIntakeOpenLoop(double output) {}

  default void stopIntake() {}
}
