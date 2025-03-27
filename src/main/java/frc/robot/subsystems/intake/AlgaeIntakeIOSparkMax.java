package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.Constants;

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {
  protected final SparkMax armMotor;
  protected final SparkFlex intakeMotor;

  protected final SparkClosedLoopController armController;
  protected final SparkClosedLoopController intakeController;

  protected double targetArmPosition;
  protected double targetIntakeVelocity;

  public AlgaeIntakeIOSparkMax(
      SparkBaseConfig armMotorConfig,
      SparkBaseConfig intakeMotorConfig) {
    armMotor = new SparkMax(Constants.AlgaeIntakeConstants.getArmMotorID(), MotorType.kBrushless);
    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    intakeMotor =
        new SparkFlex(Constants.AlgaeIntakeConstants.getIntakeMotorID(), MotorType.kBrushless);
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    armController = armMotor.getClosedLoopController();
    intakeController = intakeMotor.getClosedLoopController();

    targetArmPosition = armMotor.getEncoder().getPosition();
    targetIntakeVelocity = intakeMotor.getEncoder().getVelocity();
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.armMotorCurrent = armMotor.getOutputCurrent();
    inputs.armMotorVoltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armMotorPosition = armMotor.getEncoder().getPosition();

    inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
    inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeMotorVelocity = intakeMotor.getEncoder().getVelocity();

  }

  @Override
  public void setArmOpenLoop(double output) {
    armMotor.set(output);
  }

  @Override
  public void setArmPosition(double position) {
    targetArmPosition = position;
    armController.setReference(position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    intakeMotor.set(output);
  }

  @Override
  public void setIntakeVelocity(double velocity) {
    targetIntakeVelocity = velocity;
    intakeController.setReference(velocity, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

}
