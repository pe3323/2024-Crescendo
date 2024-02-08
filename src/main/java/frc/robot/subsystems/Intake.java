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

public class Intake extends SubsystemBase {
    private final CANSparkMax intake;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    private RelativeEncoder m_alternateEncoder;
    private double armSpeed = 0.5;
    SparkMaxPIDController pid;

    public Intake(){
        intake= new CANSparkMax(ShooterConstants.intake, MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
       intake.restoreFactoryDefaults(); 
       intake.getEncoder().setPosition(0);
 
}
    public void raise(){ //raises the roof

        intake.set(.7);
        System.out.println("Raising the arm"  +  intake.getDeviceId());

    }
                                                                                                                                                                                                                                                                                                //Calamari
    public void lower(){ //lowers the roof

      intake.set(-.7);

    }

    public void stop(){ //stops the roof    
        intake.set(0);

      

    }

    public void Station(){
        while(intake.getEncoder().getPosition()> 155){
            intake.set(-.6);
        }
         
        intake.set(0);

        while (intake.getEncoder().getPosition()< 152) {
            intake.set(.6);
        }
        
        intake.set(0);
     }

    

    public double getposition() { //sets position
        return intake.getEncoder().getPosition(); 
    }
}
