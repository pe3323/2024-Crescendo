package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
;

public class NeoShooter extends SubsystemBase implements Shooter{
    private final CANSparkMax motor1, motor2;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    private RelativeEncoder m_alternateEncoder;
    private double armSpeed = 0.5;
    SparkMaxPIDController pid;

    public NeoShooter(){
        motor1= new CANSparkMax(ShooterConstants.shooter1, MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
        motor2= new CANSparkMax(ShooterConstants.shooter2, MotorType.kBrushless);
        motor1.restoreFactoryDefaults(); 
        motor1.getEncoder().setPosition(0);
        motor1.setInverted(true);
        motor1.setIdleMode(IdleMode.kBrake);
        motor2.restoreFactoryDefaults(); 
        motor2.getEncoder().setPosition(0);
        motor2.setInverted(true);
        motor2.setIdleMode(IdleMode.kBrake);
 
    }
    public void setSpeed(double speedInpercent){ //changes speed of motors

        motor1.set(speedInpercent);
        motor2.set(speedInpercent);
        SmartDashboard.putNumber("Motor1 Speed"  +  motor1.getDeviceId(), speedInpercent);
        SmartDashboard.putNumber("Motor2 Speed"  +  motor2.getDeviceId(), speedInpercent);
        SmartDashboard.putNumber("shooter Velocity 1", motor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("shooter Velocity 2", motor2.getEncoder().getVelocity());

    }

    public double getRPM(){

        return motor2.getEncoder().getVelocity();
    }


    public void stop(){ //stops the motors  
        motor1.set(0);
        motor2.set(0);

      

    }
    

    public double getposition() { //Gets position
        return motor1.getEncoder().getPosition(); 
    }
}
