package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {
   private double targetPosition;
   private double voltage;

   @Override
    public void setArmOpenLoopVoltage(double output) {
        voltage = output;
    }
    @Override
    public void setArmOpenLoop(double output) {
        voltage = output * 12.0;
    }

    @Override
    public void setArmAngle(double angle) {
        targetPosition = angle;
    }

    @Override
    public double getArmAngleDegrees() {
        return targetPosition;
    }

    @Override
    public void stopArm() {
        // nothing
    }

    @Override
    public void setAngleDelta(double delta) {
        targetPosition += delta;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.targetPosition = targetPosition;
        inputs.atTarget = Math.abs(inputs.targetPosition - inputs.armAxisAngle) < 5.0;
        inputs.armAxisAngle = targetPosition;
        inputs.armAppliedVolts = voltage;
        inputs.atTarget = true;
    }
}
