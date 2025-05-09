package frc.robot.subsystems.climber;

public class ClimberPivotIOSim implements ClimberPivotIO {
    private double targetPosition;
    private double voltage;

    @Override
    public void setPivotOpenLoopVoltage(double output) {
        voltage = output;
    }

    @Override
    public void setPivotOpenLoop(double output) {
        voltage = output * 12.0;
    }

    @Override
    public void setPivotPositionDegrees(double position) {
        targetPosition = position;
    }

    @Override
    public double getPivotPosition() {
        return targetPosition;
    }

    @Override
    public void stopPivot() {
        // nothing
    }

    @Override
    public void setPivotVelocity(double velocity) {
        //Not implemented
    }

    @Override
    public void updateInputs(ClimberPivotIOInputs inputs) {
        inputs.targetPosition = targetPosition;
        inputs.pivotPositionDegrees = targetPosition;
        inputs.pivotAppliedVolts = voltage;
        inputs.atTarget = true;
    }

    @Override
    public void setPivotBrakeMode(com.ctre.phoenix6.signals.NeutralModeValue mode) {
        //Not implemented
    }
}