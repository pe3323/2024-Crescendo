package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class ShooterPivot extends SubsystemBase{
    private final CANSparkMax pivotMotor;
    SparkMaxPIDController pid;

    public ShooterPivot(){
        pivotMotor= new CANSparkMax(PivotConstants.pivotID, CANSparkLowLevel.MotorType.kBrushless); //makes new motor controller that is defined as the motor for the arm
        pivotMotor.restoreFactoryDefaults(); 
        pivotMotor.getEncoder().setPosition(0);

        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);
        
        
        pid = pivotMotor.getPIDController();
        pid.setP(0.05);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setFF(0.0);
        pid.setOutputRange(-0.2, 0.2);
        pid.setReference(0,CANSparkMax.ControlType.kPosition);

       // pivotMotor.
    }

    public void setPosition(double targetPosition){
       SmartDashboard.putNumber("pivotMotor Position"  +  pivotMotor.getDeviceId(), targetPosition);
       pid.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }

    public boolean isInMotion() {
        return pivotMotor.get() == 0.0;
    }


    public void stop(){ //stops the motor
        pivotMotor.set(0);
    }
}
