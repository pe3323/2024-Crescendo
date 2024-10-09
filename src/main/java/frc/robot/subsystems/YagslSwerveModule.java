package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class YagslSwerveModule {
   SwerveDrive  swerveDrive ;
    public YagslSwerveModule(){
    
        

double maximumSpeed = Units.feetToMeters(4.5);
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  try {
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
} catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
}

    }


    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }
}
