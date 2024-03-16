// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoClimber extends SubsystemBase implements Climber {
  private final CANSparkMax motor1;

  /** Creates a new ExampleSubsystem. */
  public NeoClimber(int CANId) {
      motor1 = new CANSparkMax(CANId, CANSparkLowLevel.MotorType.kBrushless); // makes new motor controller that is
      motor1.restoreFactoryDefaults();
      motor1.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
      motor1.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, 0);
  }
  
  public void raise(){    
    System.out.println( "foo");
    motor1.set(0.7);

  }

  public void lower(){
    motor1.set(-0.7);
  }

  public void stop(){
    motor1.set(0);
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
