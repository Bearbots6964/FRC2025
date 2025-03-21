package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants;

public class ClimberPivotIOSparkMax implements ClimberPivotIO {
  private final SparkMax pivotMotor;
  private final SparkBaseConfig pivotConfig;
  private final SparkAbsoluteEncoder encoder;

  private final SparkClosedLoopController controller;

  private final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5);

  public ClimberPivotIOSparkMax(SparkBaseConfig configuration) {
    this.pivotConfig = configuration;

    pivotMotor = new SparkMax(Constants.ClimberConstants.getPivotMotorID(), SparkLowLevel.MotorType.kBrushless);
    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    encoder = pivotMotor.getAbsoluteEncoder();
    controller = pivotMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ClimberPivotIOInputs inputs) {
    inputs.pivotConnected = pivotMotorConnectedDebouncer.calculate(pivotMotor.getLastError() != REVLibError.kOk);
    inputs.pivotPositionDegrees = encoder.getPosition();
    inputs.pivotVelocityDegreesPerSecond = encoder.getVelocity();
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotAppliedCurrentAmps = pivotMotor.getOutputCurrent();
  }

  @Override
  public void setPivotOpenLoop(double output) {
    pivotMotor.set(output);
  }

  @Override
  public void setPivotPosition(double position) {
    controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPivotVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  @Override
  public double getPivotPosition() {
    return encoder.getPosition();
  }

  @Override
  public void stopPivot() {
    pivotMotor.set(0);
  }
}
