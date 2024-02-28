package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class ShooterPivot extends SubsystemBase{
    private final CANSparkMax pivotMotor;
    private RelativeEncoder encoder;

    public ShooterPivot(){
        pivotMotor= new CANSparkMax(PivotConstants.pivotID, MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
        pivotMotor.restoreFactoryDefaults(); 
       // pivotMotor.getPIDController().
        encoder = pivotMotor.getEncoder();
        encoder.setPosition(0);
        pivotMotor.setInverted(true);
        
       // pivotMotor.
    }

    public void setPosition(double targetPosition){
       // encoder.(targetPosition);
        SmartDashboard.putNumber("pivotMotor Position"  +  pivotMotor.getDeviceId(), targetPosition);

    }
    public void stop(){ //stops the motor
        pivotMotor.set(0);
    }
}
