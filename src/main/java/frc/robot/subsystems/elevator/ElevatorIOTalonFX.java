package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  protected final TalonFXConfiguration leftConfig;
  protected final TalonFXConfiguration rightConfig;

  protected final TalonFX leftMotor;
  protected final TalonFX rightMotor;

  protected final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0.0);
  protected final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);
  protected final VoltageOut voltageRequest = new VoltageOut(0.0);

  protected final StatusSignal<Angle> leftMotorPosition;
  protected final StatusSignal<Angle> rightMotorPosition;
  protected final StatusSignal<AngularVelocity> leftMotorVelocity;
  protected final StatusSignal<AngularVelocity> rightMotorVelocity;
  protected final StatusSignal<Voltage> leftMotorVoltage;
  protected final StatusSignal<Voltage> rightMotorVoltage;
  protected final StatusSignal<Temperature> leftMotorTemperature;
  protected final StatusSignal<Temperature> rightMotorTemperature;
  protected final StatusSignal<Current> leftMotorCurrent;
  protected final StatusSignal<Current> rightMotorCurrent;

  private final Debouncer leftMotorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer rightMotorConnectedDebouncer = new Debouncer(0.5);

  protected AngularVelocity targetVelocity = Units.RotationsPerSecond.of(0.0);
  protected Angle targetPosition = Units.Degrees.of(0.0);

  public ElevatorIOTalonFX(TalonFXConfiguration leftConfig, TalonFXConfiguration rightConfig) {
    this.leftConfig = leftConfig;
    this.rightConfig = rightConfig;

    leftMotor = new TalonFX(Constants.ElevatorConstants.LEFT_MOTOR_CAN_ID);
    rightMotor = new TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR_CAN_ID);

    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig, 0.25));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig, 0.25));

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

    leftMotorPosition = leftMotor.getPosition();
    rightMotorPosition = rightMotor.getPosition();
    leftMotorVelocity = leftMotor.getVelocity();
    rightMotorVelocity = rightMotor.getVelocity();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    rightMotorVoltage = rightMotor.getMotorVoltage();
    leftMotorTemperature = leftMotor.getDeviceTemp();
    rightMotorTemperature = rightMotor.getDeviceTemp();
    leftMotorCurrent = leftMotor.getTorqueCurrent();
    rightMotorCurrent = rightMotor.getTorqueCurrent();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leftStatus =
        BaseStatusSignal.refreshAll(
            leftMotorPosition,
            leftMotorVelocity,
            leftMotorVoltage,
            leftMotorTemperature,
            leftMotorCurrent);
    var rightStatus =
        BaseStatusSignal.refreshAll(
            rightMotorPosition,
            rightMotorVelocity,
            rightMotorVoltage,
            rightMotorTemperature,
            rightMotorCurrent);

    inputs.leftMotorConnected = leftMotorConnectedDebouncer.calculate(leftStatus.isOK());
    inputs.rightMotorConnected = rightMotorConnectedDebouncer.calculate(rightStatus.isOK());

    inputs.leftMotorPosition = leftMotorPosition.getValue();
    inputs.rightMotorPosition = rightMotorPosition.getValue();

    inputs.leftMotorVelocity = leftMotorVelocity.getValue();
    inputs.rightMotorVelocity = rightMotorVelocity.getValue();

    inputs.leftMotorVoltage = leftMotorVoltage.getValue();
    inputs.rightMotorVoltage = rightMotorVoltage.getValue();

    inputs.leftMotorTemperature = leftMotorTemperature.getValue();
    inputs.rightMotorTemperature = rightMotorTemperature.getValue();

    inputs.leftMotorCurrent = leftMotorCurrent.getValue();
    inputs.rightMotorCurrent = rightMotorCurrent.getValue();

    inputs.targetVelocity = targetVelocity;
    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setOpenLoop(double output) {
    rightMotor.set(output);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    motionMagicVelocityRequest.withVelocity(velocity);
    rightMotor.setControl(motionMagicVelocityRequest);
  }

  @Override
  public void setPosition(Angle position) {
    targetPosition = position;
    motionMagicPositionRequest.withPosition(position);
    rightMotor.setControl(motionMagicPositionRequest);
  }

  @Override
  public void stop() {
    rightMotor.setControl(new StaticBrake());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    voltageRequest.withOutput(voltage);
    rightMotor.setControl(voltageRequest);
  }
}
