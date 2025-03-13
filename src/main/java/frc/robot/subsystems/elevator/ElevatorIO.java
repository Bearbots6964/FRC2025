package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = false;
    public double leftMotorTemperatureCelsius = 0.0;
    public double leftMotorPositionRotations = 0.0;
    public double leftMotorVelocity = 0.0;
    public double leftMotorVoltage = 0.0;
    public double leftMotorCurrent = 0.0;

    public boolean rightMotorConnected = false;
    public double rightMotorTemperatureCelsius = 0.0;
    public double rightMotorPositionRotations = 0.0;
    public double rightMotorVelocity = 0.0;
    public double rightMotorVoltage = 0.0;
    public double rightMotorCurrent = 0.0;

    public double targetVelocity = 0.0;
    public double targetPosition = 0.0;

    public boolean atTarget = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setVelocity(double velocity) {}

  default void setPosition(double position) {}

  default void setPositionDelta(double delta) {}

  default void setVoltage(double voltage) {}

  default double getDistanceFromGoal() {
    return 0.0;
  }

  default void stop() {}

  default void setSoftLimitsEnabled(boolean enabled) {}

  default void zero() {}
}
