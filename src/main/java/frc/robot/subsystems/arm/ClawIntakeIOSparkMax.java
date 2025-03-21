package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Constants;

public class ClawIntakeIOSparkMax implements ClawIntakeIO {
  protected SparkMax flywheelMotor;
  protected SparkBaseConfig flywheelConfiguration;

  public ClawIntakeIOSparkMax(SparkBaseConfig flywheelConfiguration) {
    this.flywheelConfiguration = flywheelConfiguration;

    flywheelMotor =
        new SparkMax(
            Constants.FlywheelConstants.getFlywheelMotorID(), SparkLowLevel.MotorType.kBrushless);
    tryUntilOk(
        flywheelMotor,
        5,
        () ->
            flywheelMotor.configure(
                flywheelConfiguration,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelAppliedCurrent = flywheelMotor.getOutputCurrent();
    inputs.flywheelAppliedVoltage =
        flywheelMotor.getAppliedOutput() * flywheelMotor.getBusVoltage();
    inputs.limitSwitchPressed = flywheelMotor.getForwardLimitSwitch().isPressed();
  }

  @Override
  public void setFlywheelOpenLoop(double output) {
    flywheelMotor.set(output);
  }

  @Override
  public void stopFlywheel() {
    flywheelMotor.set(0);
  }
}
