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

  public AlgaeIntakeIOSparkMax(SparkBaseConfig armMotorConfig, SparkBaseConfig intakeMotorConfig) {
    System.out.println("│╠╦ Constructing algae intake I/O!");
    double initializeTime = System.currentTimeMillis();
    System.out.print("│║╠ Creating mechanism motor... ");
    armMotor = new SparkMax(Constants.AlgaeIntakeConstants.getArmMotorID(), MotorType.kBrushless);
    System.out.println("done.");
    System.out.print("│║╠ Configuring mechanism motor... ");
    tryUntilOk(
        armMotor,
        5,
        () ->
            armMotor.configure(
                armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    System.out.println("done.");
    System.out.print("│║╠ Creating intake motor... ");
    intakeMotor =
        new SparkFlex(Constants.AlgaeIntakeConstants.getIntakeMotorID(), MotorType.kBrushless);
    System.out.println("done.");
    System.out.print("│║╠ Configuring intake motor... ");
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    System.out.println("done.");

    System.out.print("│║╠ Configuring arm and intake controllers... ");
    armController = armMotor.getClosedLoopController();
    intakeController = intakeMotor.getClosedLoopController();
    System.out.println("done.");

    System.out.print("│║╠ Getting positions... ");
    targetArmPosition = armMotor.getEncoder().getPosition();
    targetIntakeVelocity = intakeMotor.getEncoder().getVelocity();
    System.out.println("done.");

    System.out.println("│╠╝ Algae intake I/O initialized in " + String.format("%.3f", (System.currentTimeMillis() - initializeTime) * 1000.0) + "ms");
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
