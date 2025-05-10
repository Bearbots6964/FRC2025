package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class WinchIOSim implements WinchIO {
    private double targetPosition;
    private double voltage;
    private double velocity;
    private double currentPosition;

    @Override
    public void setWinchOpenLoopVoltage(double output) {
        voltage = output;
    }

    @Override
    public void setWinchOpenLoop(double output) {
        voltage = output * 12.0;
    }

    @Override
    public void setWinchPosition(double position) {
        targetPosition = position;
    }

    @Override
    public void setWinchVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public double getWinchPosition() {
        return currentPosition;
    }

    @Override
    public void stopWinch() {
        voltage = 0.0;
        velocity = 0.0;
    }

    @Override
    public void setWinchBrakeMode(NeutralModeValue mode) {
        // Not implemented
    }

    @Override
    public void updateInputs(WinchIOInputs inputs) {
        inputs.targetPosition = targetPosition;
        inputs.winchPositionDegrees = currentPosition;
        inputs.winchAppliedVolts = voltage;
        inputs.winchVelocityDegreesPerSecond = velocity;
        inputs.atTarget = true;

    }
}