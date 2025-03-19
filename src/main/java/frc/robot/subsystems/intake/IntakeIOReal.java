package frc.robot.subsystems.intake;

import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Constants;

import static frc.robot.util.SparkUtil.tryUntilOk;

public class IntakeIOReal implements IntakeIO {

  protected SparkMax intakeFlywheelMotor;
  protected SparkBaseConfig intakeFlywheelConfig;
  protected SparkClosedLoopController intakeFlywheelController;
  protected SparkAbsoluteEncoder intakeFlywheelEncoder;

  protected SparkMax intakeAxisMotor;
  protected SparkBaseConfig intakeAxisConfig;
  protected SparkClosedLoopController intakeAxisController;
  protected SparkAbsoluteEncoder intakeAxisEncoder;

  public IntakeIOReal(SparkBaseConfig flywheelConfig, SparkBaseConfig axisConfig) {
    this.intakeFlywheelConfig = flywheelConfig;
    this.intakeAxisConfig = axisConfig;

    intakeFlywheelMotor =
        new SparkMax(
            Constants.IntakeConstants.getIntakeFlywheelMotorID(),
            SparkLowLevel.MotorType.kBrushless);
    intakeFlywheelController = intakeFlywheelMotor.getClosedLoopController();
    intakeFlywheelEncoder = intakeFlywheelMotor.getAbsoluteEncoder();

    intakeFlywheelConfig
        .smartCurrentLimit(20)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .openLoopRampRate(5)
        .disableFollowerMode();
    tryUntilOk(
        intakeFlywheelMotor,
        5,
        () ->
            intakeFlywheelMotor.configure(
                intakeFlywheelConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));

    intakeAxisMotor =
        new SparkMax(
            Constants.IntakeConstants.getIntakeAxisMotorID(), SparkLowLevel.MotorType.kBrushless);
    intakeAxisController = intakeAxisMotor.getClosedLoopController();
    intakeAxisEncoder = intakeAxisMotor.getAbsoluteEncoder();

    intakeAxisConfig
        .smartCurrentLimit(20)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .openLoopRampRate(5)
        .disableFollowerMode();
    intakeAxisConfig.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(360);
    intakeAxisConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
    tryUntilOk(
        intakeAxisMotor,
        5,
        () ->
            intakeAxisMotor.configure(
                intakeAxisConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeFlywheelVoltage =
        intakeFlywheelMotor.getAppliedOutput() * intakeFlywheelMotor.getBusVoltage();
    inputs.intakeFlywheelCurrent = intakeFlywheelMotor.getOutputCurrent();

    inputs.intakeAxisVoltage = intakeAxisMotor.getAppliedOutput() * intakeAxisMotor.getBusVoltage();
    inputs.intakeAxisCurrent = intakeAxisMotor.getOutputCurrent();
  }

  @Override
  public void setIntakeOpenLoopVoltage(double voltage) {
    intakeFlywheelMotor.setVoltage(voltage);
  }

  @Override
  public void deployIntake() {
    intakeAxisController.setReference(45, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void retractIntake() {
    intakeAxisController.setReference(0.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stopIntake() {
    setIntakeOpenLoopVoltage(0.0);
  }
}
