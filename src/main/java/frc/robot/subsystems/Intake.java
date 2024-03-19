package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;;

public class Intake extends SubsystemBase {
    private final CANSparkMax intake;
    private DigitalInput sensor;
    SparkPIDController pidController;

    public Intake() {
        intake = new CANSparkMax(ShooterConstants.intake, MotorType.kBrushless); // makes new motor controller that is
                                                                                 // defined as the motor for the arm
        intake.restoreFactoryDefaults();
        intake.getEncoder().setPosition(0);

        pidController = intake.getPIDController();
        pidController.setP(0.1);
        pidController.setI(0.0);
        pidController.setD(0.0);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-1, 1);

        sensor = new DigitalInput(1);
    }

    public boolean HasNote() {
        return !sensor.get();

    }

    public void raise() { // raises the roof

        intake.set(-.30);
        SmartDashboard.putBoolean("Sensor Value", sensor.get());
        

    }

    // Calamari
    public void lower() { // lowers the roof

        intake.set(.1);
        

    }

    public void stop() { // stops the roof
        intake.set(0);

    }

    public double getposition() { // sets position
        return intake.getEncoder().getPosition();
    }

}
