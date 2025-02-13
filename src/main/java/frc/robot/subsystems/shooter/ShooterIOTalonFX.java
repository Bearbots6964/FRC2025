package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * IO implementation for the shooter using TalonFX (which means Falcon 500's and Krakens).
 *
 * @implNote This class may seem daunting at first - slow down and try to understand what each line does before trying
 *     to piece together what the whole thing does.
 */
public class ShooterIOTalonFX implements ShooterIO {

    // Motor configuration.
    // This is passed in via the constructor,
    // which allows us to configure the motor elsewhere; say, the Constants file.
    protected final TalonFXConfiguration flywheelConfiguration;
    // Motor object itself.
    protected final TalonFX flywheelMotor;
    /**
     * The motion magic velocity request. This specific control method directly controls acceleration of the motor
     * instead of controlling a rough velocity.
     *
     * @implNote This is set here instead of on the fly because we can reuse them fairly easily, and I don't know how
     *     nicely the JVM plays with creating a bunch of different objects and then immediately throwing them away.
     * @see <a
     *     href="https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-velocity">Motion
     *     Magic velocity control documentation</a>
     * @see <a
     *     href="https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#field-oriented-control">Field
     *     Oriented Control documentation</a>
     */
    protected final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityRequest =
            new MotionMagicVelocityTorqueCurrentFOC(0.0);
    /**
     * Input signals for the flywheel motor.
     *
     * @implNote {@link StatusSignal} is basically just a wrapper for whatever type you're using. It has some extra
     *     methods that make some things easier.
     */
    protected final StatusSignal<AngularVelocity> flywheelVelocity;

    protected final StatusSignal<Voltage> flywheelVoltage;
    protected final StatusSignal<Current> flywheelCurrent;
    /**
     * Connection debouncer, to prevent false positives.
     *
     * @see Debouncer
     */
    private final Debouncer flywheelConnectedDebouncer = new Debouncer(0.5);
    // This can be set to zero at initialization, because there isn't a goal at that point.
    protected AngularVelocity targetVelocity = Units.RotationsPerSecond.of(0.0);

    /**
     * Constructor for ShooterIOTalonFX.
     *
     * @param flywheelConfiguration Configuration for the flywheel motor.
     * @implNote We can create the motor configuration over in {@link Constants}. That kind of consolidation is what
     *     you'll want to achieve.
     */
    protected ShooterIOTalonFX(TalonFXConfiguration flywheelConfiguration) {
        // set our instance variables to whatever we were constructed with
        this.flywheelConfiguration = flywheelConfiguration;

        // instantiate motors and encoder
        flywheelMotor = new TalonFX(Constants.ShooterConstants.flywheelMotorID);

        // apply configs and reset position to 0 (it's a flywheel,
        // so position doesn't *really* matter, but it's nice to have just to make sure)
        // side note:
        // tryUntilOk is a utility method
        // that will keep trying to apply the configuration until it works, or until it's tried 5 times.
        tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfiguration, 0.25));
        tryUntilOk(5, () -> flywheelMotor.setPosition(Units.Degrees.of(0.0), 0.25));

        // status signals
        // These get refreshed every time updateInputs is called.
        // It's kinda weird.
        // I don't entirely love the way Java does references, but whatever.
        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelCurrent = flywheelMotor.getStatorCurrent();
        flywheelVoltage = flywheelMotor.getMotorVoltage();
    }

    // override the updateInputs method so we can actually, y'know, update
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // refresh all signals
        // This isn't something you're going to find in,
        // like, /any/ java code apart from FRC.
        // This is specifically a Phoenix thing.
        // I am actually not /entirely/ sure what it does,
        // but it seems important, on account of the other usages of the method.
        var flywheelStatus = BaseStatusSignal.refreshAll(flywheelCurrent, flywheelVelocity, flywheelVoltage);

        // update flywheel inputs
        // Pretty straightforward, no real notes here.
        inputs.flywheelConnected = flywheelConnectedDebouncer.calculate(flywheelStatus.isOK());
        inputs.flywheelAppliedVolts = flywheelVoltage.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        inputs.flywheelVelocity = flywheelVelocity.getValue();
    }

    /**
     * Set the flywheel to open loop control.
     *
     * @param output The output value to set.
     * @implNote See {@link Shooter#setShooterOpenLoop(double)} for information on why this is done the way it is.
     */
    @Override
    public void setShooterOpenLoop(double output) {
        flywheelMotor.set(output); // this one's pretty straightforward
        targetVelocity = Units.RotationsPerSecond.of(0.0);
    }

    /**
     * Set the flywheel velocity. Uses Motion Magic.
     *
     * @param velocity The velocity to set the flywheel to.
     * @see <a
     *     href="https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html">Motion
     *     Magic</a>
     */
    @Override
    public void setShooterVelocity(AngularVelocity velocity) {
        motionMagicVelocityRequest.withVelocity(velocity);
        flywheelMotor.setControl(motionMagicVelocityRequest);
        targetVelocity = velocity;
    }

    /** Stop the flywheel. See {@link Shooter#stop()} for more information. */
    @Override
    public void stop() {
        flywheelMotor.set(0.0);
        targetVelocity = Units.RotationsPerSecond.of(0.0);
    }

    /**
     * Helper method for {@link ShooterIOTalonFXSim}.
     *
     * @return The motor's simulation state.
     */
    protected TalonFXSimState getSimState() {
        return flywheelMotor.getSimState();
    }
}
