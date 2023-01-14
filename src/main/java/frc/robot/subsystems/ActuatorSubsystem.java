package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ActuatorSubsystem extends SubsystemBase {
    //private Talon motor;
    private Servo actuatorMotor;

    public  ActuatorSubsystem() {
//actuatorMotor = new motor(constant)
    actuatorMotor = new Servo(Constants.ACTUATOR_MOTOR);

    }

    public void actuatorExtend() { 
        actuatorMotor.set(Constants.ACTUATOR_EXTEND);

    }


    public void  actuatorRetract() {
        actuatorMotor.set(Constants.ACTUATOR_RETRACT);


    }

    public void actuatorStop(){
        actuatorMotor.set(Constants. ACTUATOR_STOP);
        //motor.set(off)
     }


    public void  ActuatorRetract() {


    }

    public void ActuatorStop(){
        //motor.set(off)
     }
 }