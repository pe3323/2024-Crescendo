// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonClimber extends SubsystemBase implements Climber {
  private final TalonSRX motor1;

  /** Creates a new ExampleSubsystem. */
  public TalonClimber(int CANId) {
      motor1 = new TalonSRX(CANId); // makes new motor controller that is
      motor1.configFactoryDefault();
  }
  
  public void raise(){
    System.out.println( "foo");
    motor1.set(TalonSRXControlMode.PercentOutput, .7);

  }

  public void lower(){
    motor1.set(TalonSRXControlMode.PercentOutput, -.7);
  }

  public void stop(){
    motor1.set(TalonSRXControlMode.PercentOutput, 0);
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
