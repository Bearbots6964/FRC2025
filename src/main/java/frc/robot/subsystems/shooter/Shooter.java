package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Some thoughts:
 *
 * <ul>
 *   <li>You'll see throughout this subsystem and its related classes that we have two main ways of controlling the
 *       shooter: open loop and closed loop. Open loop control is when we just give the motor a number between -1 and 1.
 *       During that, the motor will spin at that percent speed (well, technically it's how many times the motor's max
 *       output it should run at, but it's easier to call it a percentage). Closed-loop takes a specific velocity and
 *       tries to keep the motor spinning at that speed using various state-space controllers. It does all those
 *       calculations on the motor, so it's usually a lot lower latency than if we did the traditional "run everything
 *       on the RIO" method.
 *   <li>SysID, short for System Identification, is a method of figuring out some constants about, for instance, how
 *       much energy it takes to overcome static friction in a motor. In /most/ cases, you won't need to worry about it,
 *       and if it suddenly becomes incredibly mission-critical, you can probably write it in under 15 minutes.
 *   <li>Documentation throughout the libraries we use is... lacking, to say the least. I hope to mitigate this by
 *       writing a good, comprehensible subsystem and extensively documenting everything you'll need to know to do the
 *       same.
 * </ul>
 */
public class Shooter extends SubsystemBase {

    // The IO instance for the shooter
    private final ShooterIO io;
    // These are the inputs.
    // If it's showing as a nonexistent class,
    // make sure you have the @AutoLog annotation in the ShooterIOInputs class.
    // After, compile the code and it should work.
    // (The reason for this is
    // that there's an annotation processor that generates a class for us that does what we need it to.)
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    // An alert. This can be seen on the dashboard (although you'll have to add the widget).
    // This is really helpful for debugging.
    private final Alert flywheelDisconnectedAlert;
    // The system identification routine for the shooter.
    private final SysIdRoutine sysId;

    /**
     * Constructor for the shooter.
     *
     * @param io The IO instance for the shooter.
     * @implNote The ShooterIO object is instantiated in the {@link frc.robot.RobotContainer robot container}. Over
     *     there, we determine what kind of IO we want to use (real robot, simulated robot, or a log replay).
     */
    public Shooter(ShooterIO io) {
        this.io = io;
        flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor.", Alert.AlertType.kError);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Units.Volts)), null, this));
    }

    /** Periodic function. I'm sure you've seen the boilerplate comment before. Every 20 ms, this function is called. */
    public void periodic() {
        // Update and log the inputs
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Stop the shooter if the robot is disabled
        if (DriverStation.isDisabled()) {
            io.stop();
        }

        // Alert if the flywheel is disconnected
        flywheelDisconnectedAlert.set(!inputs.flywheelConnected && Constants.getCurrentMode() != Mode.SIM);
    }

    /**
     * Sets the shooter open loop value (from -1.0 to 1.0, with each extreme representing the maximum power the motor
     * can put out).
     */
    public void setShooterOpenLoop(double output) {
        io.setShooterOpenLoop(output);
    }

    /**
     * Sets the shooter velocity. This takes angular velocity as an argument. In my opinion, you'll almost always want
     * to use {@link Units units} for everything that requires some number to operate when that number has any sort of
     * unit. It makes the code a lot more readable, and eliminates the chance of unit discrepancies. (I believe unit
     * discrepancies caused that one rocket to blow up, so the most obvious logical next step is to beat NASA.)
     */
    public void setShooterVelocity(AngularVelocity velocity) {
        io.setShooterVelocity(velocity);
    }

    /** Stops the shooter. 'Nuff said. */
    public void stop() {
        io.stop();
    }

    // ###### System Identification ######

    /**
     * Runs the shooter with the specified output in the context of a system identification routine.
     *
     * @implNote The primary reason we're using this instead of calling the {@link Shooter#setShooterOpenLoop(double)}
     *     method directly is that this provides a convenient way to update what system identification is doing (say,
     *     changing characterization to use some other motor, and therefore not having to update multiple
     *     {@link ShooterIO} implementation methods).
     */
    private void runCharacterization(double output) {
        io.setShooterOpenLoop(output);
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     *
     * @see <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">the
     *     WPILib System Identification docs</a>
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     *
     * @see <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html">the
     *     WPILib System Identification docs</a>
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    // ###### Command Factories ######

    /**
     * Returns a command that runs the shooter open-loop with the specified output.
     *
     * @implNote Command factories make life so, so much easier. They're a great way to create commands to do something
     *     without making every method in the subsystem public (and thus implementing encapsulation, etc., etc.). It
     *     enables a set-and-forget trigger back over in RobotContainer:<br>
     *     {@code controller.a().onTrue(shooter.runOpenLoop { controller.leftX })} <br>
     *     which is much easier than some multipart lambda-whatever-function that you'd have to write otherwise. It also
     *     makes RobotContainer a bit cleaner.
     */
    public Command runManually(DoubleSupplier speed) {
        return run(() -> setShooterOpenLoop(speed.getAsDouble()));
    }
}
