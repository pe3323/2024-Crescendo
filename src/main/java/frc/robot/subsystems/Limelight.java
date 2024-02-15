// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class Limelight extends SubsystemBase {

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tid;
  /** Creates a new ExampleSubsystem. */

  public Limelight() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tid = table.getEntry("tid");

    // read values periodically

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void getValues() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double id = tid.getDouble(0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("AprilTagID", id);
    SmartDashboard.putNumber("distancetoApT", getDistanceToTag((int) id));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getAngle() {
    double a1 = Constants.VisionConstants.cameraAngle;
    double a2 = ty.getDouble(0.0);
    double totalAngle = a1 + a2;

    return (Math.PI / 180.0) * totalAngle;
  }

  public double getDistanceToTag(int tagID) {
    if (getClosestAprilTag() == tagID) {

      return (Constants.VisionConstants.apriltagHeight - Constants.VisionConstants.cameraHeight) / Math.tan(getAngle());

    }
    ;

    return 0.0;
  }

  public int getClosestAprilTag() {
    return (int) tid.getInteger(-1);
  }

  public void turnToTarget(SwerveSubsystem swerveSubsystem) {

    while (tx.getDouble(0.0) > .5) {
      SwerveJoystickCmd Joystick = new SwerveJoystickCmd(
          swerveSubsystem,
          () -> 0.0,
          () -> 0.0,
          () -> -1.0,
          () -> true);
          Joystick.execute(); 
    }

  }
 public double getTx(){
  return tx.getDouble(0.0);
 }
}
