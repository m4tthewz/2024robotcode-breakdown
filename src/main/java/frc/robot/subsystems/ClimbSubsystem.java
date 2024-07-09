package frc.robot.subsystems;

// import needed libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/*
initalizes, declares and moniters climb motor and encoder
*/

//puts all info into the climbSusystem in the Subsystem Base (organization)
public class ClimbSubsystem extends SubsystemBase {

    //declare the motor for the climber and the encoder
    CANSparkMax ClimbSpinMotor;
    RelativeEncoder ClimbEncoder;

    // used for logging climber speed and encoder (found from getSpin and getEncoder) for monitering
    private final String CLIMBER_SPEED_LOG_PATH = "/Climber/Speeds";
    private final String CLIMBER_ENCODER_LOG_PATH = "/Climber/Encoder";

    // intalizes motor and encoder 
    public ClimbSubsystem() {
        //initialize the climber motor using CAN ID from ClimbConstants in Constants
        ClimbSpinMotor = new CANSparkMax(ClimbConstants.ClimbSpinMotorCanID, MotorType.kBrushless);
        ClimbEncoder = ClimbSpinMotor.getEncoder();

    }

    // function to set speed of the motor
    public void setSpin(double speed) {
        ClimbSpinMotor.set(speed);
    }

    // function to get current speed of the climber motor
    public double getSpin(){
        return ClimbSpinMotor.get();
    }

    // function to get current position of the encoder
    public double getEncoder(){
        return ClimbEncoder.getPosition();
    }

    // add current speed and position to log
    public void logOutputs(){
        Logger.recordOutput(getName() + CLIMBER_SPEED_LOG_PATH, getSpin());
        Logger.recordOutput(getName() + CLIMBER_ENCODER_LOG_PATH, getEncoder());
    }

}
