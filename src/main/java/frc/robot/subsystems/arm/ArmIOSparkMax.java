package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import frc.robot.Constants.SuperstructureConstants;

// TODO: MotionMagic
public class ArmIOSparkMax implements ArmIO {
  protected SparkMax armMotor;
  protected SparkBaseConfig armConfiguration;
  protected SparkClosedLoopController armController;
  protected SparkAbsoluteEncoder armEncoder;
  protected double targetPosition;

  public ArmIOSparkMax(SparkBaseConfig armConfiguration) {
    this.armConfiguration = armConfiguration;

    armMotor =
        new SparkMax(
            SuperstructureConstants.ArmConstants.getArmAxisMotorID(),
            SparkLowLevel.MotorType.kBrushless);
    armController = armMotor.getClosedLoopController();
    armEncoder = armMotor.getAbsoluteEncoder();

    // tryUntilOk(5, () -> armAxisMotor.getConfigurator().apply(armAxisConfiguration, 0.25));
    // tryUntilOk(5, () -> armAxisMotor.setPosition(Units.Degrees.of(0.0), 0.25));

    armConfiguration
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(180.0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0.0);
    armConfiguration.openLoopRampRate(1).disableFollowerMode();
    armConfiguration
        .closedLoop
        .p(0.015)
        .i(0)
        .d(0)
        .maxOutput(0.5)
        .minOutput(-0.5)
        .positionWrappingEnabled(true);
    armConfiguration.absoluteEncoder.positionConversionFactor(360);
    armConfiguration
        .absoluteEncoder
        .velocityConversionFactor(360)
        .zeroOffset(Units.Rotations.of(0.6170204).in(Units.Rotations));
    armConfiguration.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                armConfiguration,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));

    targetPosition = 170.0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAxisAngle = getArmAngleDegrees();
    inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armAppliedCurrentAmps = armMotor.getOutputCurrent();
    inputs.armVelocity = armMotor.getAbsoluteEncoder().getVelocity();
    inputs.targetPosition = targetPosition;

    inputs.atTarget = Math.abs(armMotor.getAbsoluteEncoder().getPosition() - targetPosition) < 5.0;
  }

  @Override
  public void setArmOpenLoopVoltage(double output) {
    armMotor.setVoltage(output);
  }

  @Override
  public void setArmOpenLoop(double output) {
    armMotor.set(output);
  }

  @Override
  public void setArmAngle(double angle) {
    // armAxisMotor.setControl(motionMagic.withPosition(angle));
    targetPosition = angle;
    armMotor
        .getClosedLoopController()
        .setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public double getArmAngleDegrees() {
    return armMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void stopArm() {
    armMotor.setVoltage(0);
  }

  @Override
  public void setAngleDelta(double delta) {
    setArmAngle(targetPosition + delta);
  }

  @Override
  public double getDistanceFromGoal() {
    return Math.abs(armMotor.getAbsoluteEncoder().getPosition() - targetPosition);
  }
}
