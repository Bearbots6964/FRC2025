package frc.robot.subsystems.arm;

public class ClawIntakeIOSim implements ClawIntakeIO {
  private double voltage;

    @Override
  public void setIntakeOpenLoop(double output) {
    voltage = output * 12.0;
  }

  @Override
  public void stopIntake() {
    voltage = 0.0;
  }

    @Override
  public void updateInputs(ClawIntakeIOInputs inputs) {
    inputs.intakeAppliedVoltage = voltage;
    inputs.thingGripped = true;
  }
}