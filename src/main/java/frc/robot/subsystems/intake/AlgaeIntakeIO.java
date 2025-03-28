package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public class AlgaeIntakeIOInputs {
    public double armMotorPosition = 0.0;
    public double armMotorVoltage = 0.0;
    public double armMotorCurrent = 0.0;

    public double intakeMotorVelocity = 0.0;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorCurrent = 0.0;

    public boolean extended = false;
  }

  default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  default void setArmOpenLoop(double output) {}

  default void setArmPosition(double position) {}

  default void setIntakeOpenLoop(double output) {}

  default void setIntakeVelocity(double velocity) {}
}
