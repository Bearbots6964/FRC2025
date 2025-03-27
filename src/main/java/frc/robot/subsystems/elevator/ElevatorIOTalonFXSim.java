package frc.robot.subsystems.elevator;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotContainer;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {

  private final DCMotor motors = DCMotor.getKrakenX60(2);
  private final ElevatorSim sim = new ElevatorSim(0.11971, 0.0031455, motors, 0, inchesToMeters(31), true, 0, null);

  private final TalonFXSimState leftMotorSim = new TalonFXSimState(leftMotor);
  private final TalonFXSimState rightMotorSim = new TalonFXSimState(rightMotor);

  public ElevatorIOTalonFXSim(TalonFXConfiguration leftMotor, TalonFXConfiguration rightMotor) {
    super(leftMotor, rightMotor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    super.updateInputs(inputs);
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInput(inputs.leftMotorVoltage, inputs.rightMotorVoltage);
    sim.update(0.02);

    inputs.rightMotorPositionRotations = sim.getPositionMeters() * 3.637;
    inputs.leftMotorPositionRotations = sim.getPositionMeters() * 3.637;
    inputs.leftMotorVelocity = sim.getVelocityMetersPerSecond() * 3.637;
    inputs.rightMotorVelocity = sim.getVelocityMetersPerSecond() * 3.637;
    inputs.leftMotorVoltage = leftMotorSim.getMotorVoltage();
    inputs.rightMotorVoltage = rightMotorSim.getMotorVoltage();
    inputs.leftMotorCurrent = leftMotorSim.getSupplyCurrent();
    inputs.rightMotorCurrent = rightMotorSim.getSupplyCurrent();
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
    inputs.leftMotorTemperatureCelsius = Double.MAX_VALUE; // kraken love heat
    inputs.rightMotorTemperatureCelsius = Double.MAX_VALUE;
  }

}
