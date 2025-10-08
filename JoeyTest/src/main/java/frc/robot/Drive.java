package frc.robot; 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drive {
 public static final double SWERVE_DIST_FROM_CENTER = 0.3254375;
 public static final Translation2d centerLocation     = new Translation2d(0, 0);
 public static final Translation2d frontLeftLocation  = new Translation2d(SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final Translation2d frontRightLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);
 public static final Translation2d backLeftLocation   = new Translation2d(-SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final Translation2d backRightLocation  = new Translation2d(-SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);


 public SwerveDriveKinematics swerveDriveKinematics;

 private SwerveModule frontLeft;
 private SwerveModule frontRight;
 private SwerveModule backLeft;
 private SwerveModule backRight;

    public Drive() {
        swerveDriveKinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
        );
        
        frontLeft  = new SwerveModule(14, 15, false);
        frontRight = new SwerveModule(16, 17, true);
        backLeft   = new SwerveModule(12, 13, false);
        backRight  = new SwerveModule(10, 11, true);

    
    }

   

    public void teleopDrive(double forwardPower, double strafePower, double rotatePower, boolean fieldDrive){
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(forwardPower, strafePower, rotatePower)); 
              
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);  
    }

}
