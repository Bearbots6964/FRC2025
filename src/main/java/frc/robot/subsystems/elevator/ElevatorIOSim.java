package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
  private double targetPosition;
  private double voltage;
  private double velocity;

  @Override
  public void setOpenLoop(double output) {
    voltage = output * 12.0;
  }

  @Override
  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }

  @Override
  public void setPosition(double position) {
    targetPosition = position;
  }

  @Override
  public void setPositionDelta(double delta) {
    targetPosition += delta;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public void stop() {
    voltage = 0.0;
    velocity = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.targetPosition = targetPosition;
    inputs.leftMotorPositionRotations = targetPosition;
    inputs.rightMotorPositionRotations = targetPosition;
    inputs.leftMotorVoltage = voltage;
    inputs.rightMotorVoltage = voltage;
    inputs.leftMotorVelocity = velocity;
    inputs.rightMotorVelocity = velocity;
    inputs.atTarget = true;
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    // TODO Auto-generated method stub
  }

  @Override
  public void zero() {
    // TODO Auto-generated method stub
  }
}