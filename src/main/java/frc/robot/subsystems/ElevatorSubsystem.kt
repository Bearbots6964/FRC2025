package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

object ElevatorSubsystem : SubsystemBase() {

    private val id = Constants.OperatorConstants.ELEVATOR_MOTOR_ID;
    private val motor = TalonFX(id, "Elevator")

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


    fun spin(amplitude: Double){
        motor.set(amplitude)
    }

    fun stop(){
        motor.stopMotor()
    }

    fun setPos(pos: Double){
        //motor.
    }

    fun feed(){
        motor.feed()
    }

    fun setVoltage(voltage: Double){
        motor.setVoltage(voltage)
    }
}