package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.SuperstructureConstants.ClawIntakeConstants;

public class ClawIntakeIOSparkMax implements ClawIntakeIO {
  protected SparkMax intakeMotor;
  protected SparkBaseConfig intakeConfiguration;
  protected final Debouncer gripDebouncer = new Debouncer(0.5);

  public ClawIntakeIOSparkMax(SparkBaseConfig intakeConfiguration) {
    System.out.println("│╠╦ Constructing claw intake I/O!");
    double initializeTime = System.currentTimeMillis();
    System.out.print("│║╠ Assigning configs to self... ");
    this.intakeConfiguration = intakeConfiguration;
    System.out.println("done.");

    System.out.print("│║╠ Creating mechanism motor... ");
    intakeMotor =
        new SparkMax(
            SuperstructureConstants.ClawIntakeConstants.getClawMotorID(),
            SparkLowLevel.MotorType.kBrushless);
    System.out.println("done.");
    System.out.print("│║╠ Configuring mechanism motor... ");
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfiguration,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    System.out.println("done.");


    System.out.println("│╠╝ Claw intake I/O initialized in " + String.format("%.3f", (System.currentTimeMillis() - initializeTime) * 1000.0) + "ms (note: no logging of algae intake subsystem initialization)");
  }

  @Override
  public void updateInputs(ClawIntakeIOInputs inputs) {
    inputs.intakeAppliedCurrent = intakeMotor.getOutputCurrent();
    inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.limitSwitchPressed = intakeMotor.getForwardLimitSwitch().isPressed();
    inputs.thingGripped =
        gripDebouncer.calculate(
            intakeMotor.getOutputCurrent() > ClawIntakeConstants.clawGrippedCurrent
                || intakeMotor.getForwardLimitSwitch().isPressed());
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    intakeMotor.set(output);
  }

  @Override
  public void stopIntake() {
    intakeMotor.set(0);
  }
}
