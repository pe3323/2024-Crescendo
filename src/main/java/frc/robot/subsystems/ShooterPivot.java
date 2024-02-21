package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class ShooterPivot extends SubsystemBase{
    private final CANSparkMax pivotMotor;

    public ShooterPivot(){
        pivotMotor= new CANSparkMax(PivotConstants.pivotID, MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
        pivotMotor.restoreFactoryDefaults(); 
        pivotMotor.getEncoder().setPosition(0);
    }

    public void setSpeed(double speedInpercent){
        pivotMotor.set(speedInpercent);
        SmartDashboard.putNumber("pivotMotor Speed"  +  pivotMotor.getDeviceId(), speedInpercent);

    }
    public void stop(){ //stops the motor
        pivotMotor.set(0);
    }
}
