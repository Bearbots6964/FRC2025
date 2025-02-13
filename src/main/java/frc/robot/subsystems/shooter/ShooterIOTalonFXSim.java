package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control. Chances are you won't need to read too deep into this, but it's not
 * too complicated, just really wordy.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html">WPILib
 *     Physics Sim docs</a>
 */
public class ShooterIOTalonFXSim extends ShooterIOTalonFX {
    // Sim state for the motor.
    private final TalonFXSimState flywheelMotorSim;
    // Actual mechanism simulation.
    private final FlywheelSim flywheelSim;

    /**
     * Creates a new ShooterIOTalonFXSim.
     *
     * @param flywheelConfiguration The configuration for the flywheel motor.
     * @implNote This should be relatively similar to the real IO implementation, {@link ShooterIOTalonFX}. (I mean,
     *     after all, it is overriding it.)
     */
    public ShooterIOTalonFXSim(TalonFXConfiguration flywheelConfiguration) {
        super(flywheelConfiguration);
        // If you read through the comments for ShooterIOTalonFX,
        // you should have a general idea of why we call this.
        flywheelMotorSim = super.getSimState();

        // The actual mechanism simulation instantiation. I'll try to explain this as best I can.
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem( // Flywheel mechanism state-space model
                        DCMotor.getKrakenX60Foc(
                                1), // DCMotor is basically a set of constants that define a motor's behavior.
                        Constants.ShooterConstants.flywheelMomentOfInertia, // MOI is important
                        // for dictating how long it takes for the mechanism to spin up or slow down.
                        Constants.ShooterConstants.flywheelGearing), // Gearbox
                DCMotor.getKrakenX60(1)); // IDK what this does
    }

    // We don't need an entirely different implementation for the updateInputs method,
    // but there are some simulation-specific things we need to update once per loop.
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        super.updateInputs(inputs);
        flywheelSim.setInputVoltage(RobotController.getBatteryVoltage());
        flywheelSim.update(0.02);
        flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity());
    }

    // Everything else just gets inherited from the parent class.
}
