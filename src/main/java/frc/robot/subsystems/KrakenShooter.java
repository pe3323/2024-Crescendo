package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
;

public class KrakenShooter extends SubsystemBase implements Shooter {
    private final TalonFX motor1, motor2;
    private static final int kCPR = 8192;
    private RelativeEncoder m_alternateEncoder;
    private double armSpeed = 0.5;
    SparkMaxPIDController pid;

    public KrakenShooter(){
        motor1= new TalonFX(ShooterConstants.shooter1); //makes new motor controller that is defined as the motor for the arm
        motor2= new TalonFX(ShooterConstants.shooter2);
        motor1.getConfigurator().apply(new TalonFXConfiguration());
        motor1.getConfigurator().setPosition(0);
        motor1.setInverted (true);
        motor2.getConfigurator().apply(new TalonFXConfiguration());
        motor2.getConfigurator().setPosition(0);
 
    }
    public void setSpeed(double speedInpercent){ //changes speed of motors

        motor1.set(speedInpercent);
        motor2.set(speedInpercent);
        SmartDashboard.putNumber("Motor1 Speed"  +  motor1.getDeviceID(), speedInpercent);
        SmartDashboard.putNumber("Motor2 Speed"  +  motor2.getDeviceID(), speedInpercent);

    }

    public void stop(){ //stops the motors  
        motor1.set(0);
        motor2.set(0);

      

    }
    

    public double getposition() { //Gets position
        return motor1.getPosition().getValueAsDouble()  ; 
    }
}
