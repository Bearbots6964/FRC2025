package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  protected final TalonFXS pivotMotor;
  protected final TalonFXSConfiguration pivotConfig;

  protected final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);
  protected final VoltageOut voltageRequest = new VoltageOut(0.0);
  protected final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  protected final StatusSignal<Angle> pivotMotorPosition;
  protected final StatusSignal<AngularVelocity> pivotMotorVelocity;
  protected final StatusSignal<Voltage> pivotMotorVoltage;
  protected final StatusSignal<Current> pivotMotorCurrent;

  protected final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5);

  protected double targetPosition = 0.0;
  protected boolean atTarget = false;

  public ArmIOTalonFX(TalonFXSConfiguration configuration) {
    this.pivotConfig = configuration;

    pivotMotor = new TalonFXS(Constants.ArmConstants.getArmAxisMotorID());

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));

    pivotMotorPosition = pivotMotor.getPosition();
    pivotMotorVelocity = pivotMotor.getVelocity();
    pivotMotorVoltage = pivotMotor.getMotorVoltage();
    pivotMotorCurrent = pivotMotor.getSupplyCurrent();

    targetPosition = pivotMotorPosition.getValue().in(Units.Degrees);

    // if abs(motor position) > 1, mod by 1 or -1
    if (Math.abs(pivotMotorPosition.getValue().in(Units.Rotations)) > 1) {
      pivotMotor.setPosition(pivotMotorPosition.getValue().in(Units.Rotations) % (pivotMotorPosition.getValue().in(Units.Rotations) > 0 ? 1 : -1));
    }
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    var pivotStatus =
        BaseStatusSignal.refreshAll(
            pivotMotorPosition, pivotMotorVelocity, pivotMotorVoltage, pivotMotorCurrent);

    inputs.armAxisAngle = pivotMotorPosition.getValue().in(Units.Degrees);
    inputs.armAppliedVolts = pivotMotorVoltage.getValue().in(Units.Volts);
    inputs.armAppliedCurrentAmps = pivotMotorCurrent.getValue().in(Units.Amps);
    inputs.armVelocity = pivotMotorVelocity.getValue().in(Units.Degrees.per(Units.Minute));
    inputs.targetPosition = targetPosition;
    inputs.atTarget = atTarget;
  }

  @Override
  public void setArmOpenLoopVoltage(double output) {
    voltageRequest.withOutput(output);
    pivotMotor.setControl(voltageRequest);
  }

  @Override
  public void setArmOpenLoop(double output) {
    pivotMotor.set(output);
  }

  @Override
  public void setArmAngle(double angle) {
    motionMagicPositionRequest.withPosition(Units.Degrees.of(angle));
    pivotMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public double getArmAngleDegrees() {
    return pivotMotorPosition.getValue().in(Units.Degrees);
  }

  @Override
  public void stopArm() {
    pivotMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public void setAngleDelta(double delta) {
    targetPosition += delta;
    motionMagicPositionRequest.withPosition(Units.Degrees.of(targetPosition));
    pivotMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public double getDistanceFromGoal() {
    return Math.abs(pivotMotorPosition.getValue().in(Units.Degrees) - targetPosition);
  }
}
