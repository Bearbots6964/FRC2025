package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public double flywheelAppliedVoltage = 0.0;
    public double flywheelAppliedCurrent = 0.0;
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setFlywheelOpenLoop(double output) {}

  default void stopFlywheel() {}

}
