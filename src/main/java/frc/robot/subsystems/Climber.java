// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Climber extends SubsystemBase {
  private final VictorSPX motor1;
  private final VictorSPX motor2;

  /** Creates a new ExampleSubsystem. */
  public Climber() {
      motor1 = new VictorSPX(ShooterConstants.intake); // makes new motor controller that is
      motor1.configFactoryDefault();
      motor2 = new VictorSPX(ShooterConstants.intake); // makes new motor controller that is
      motor2.configFactoryDefault();
      

  }
  
  public void raise(){
    motor1.set(VictorSPXControlMode.PercentOutput, .7);
    motor2.set(VictorSPXControlMode.PercentOutput, .7);
  }

  public void lower(){
    motor1.set(VictorSPXControlMode.PercentOutput, -.7);
    motor2.set(VictorSPXControlMode.PercentOutput, -.7);
  }

  public void stop(){
    motor1.set(VictorSPXControlMode.PercentOutput, 0);
    motor2.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
