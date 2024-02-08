package frc.robot.subsystems;

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

public class Shooter extends SubsystemBase {
    private final CANSparkMax shooter;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    private RelativeEncoder m_alternateEncoder;
    private double armSpeed = 0.5;
    SparkMaxPIDController pid;

    public Shooter(){
        shooter= new CANSparkMax(ShooterConstants.shooter, MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
      shooter.restoreFactoryDefaults(); 
       shooter.getEncoder().setPosition(0);
 
}
    public void raise(){ //raises the roof

        shooter.set(.7);
        System.out.println("Raising the arm"  +  shooter.getDeviceId());

    }
                                                                                                                                                                                                                                                                                                //Calamari
    public void lower(){ //lowers the roof

      shooter.set(-.7);

    }

    public void stop(){ //stops the roof    
        shooter.set(0);

      

    }

    public void Station(){
        while(shooter.getEncoder().getPosition()> 155){
            shooter.set(-.6);
        }
         
        shooter.set(0);
       
        while (shooter.getEncoder().getPosition()< 152) {
            shooter.set(.6);
        }
        
        shooter.set(0);
     }

    

    public double getposition() { //sets position
        return shooter.getEncoder().getPosition(); 
    }
}
