package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setVelocity(double velocity) {}

  default void setPosition(double position) {}

  default void setVoltage(double voltage) {}

  default void stop() {}

}
