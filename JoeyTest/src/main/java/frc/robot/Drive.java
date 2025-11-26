package frc.robot; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;



public class Drive {
 public static final double SWERVE_DIST_FROM_CENTER = 0.3254375;
 public static final Translation2d centerLocation     = new Translation2d(0, 0);
 public static final Translation2d frontLeftLocation  = new Translation2d(SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final Translation2d frontRightLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);
 public static final Translation2d backLeftLocation   = new Translation2d(-SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final Translation2d backRightLocation  = new Translation2d(-SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);


 public SwerveDriveKinematics swerveDriveKinematics;
 public SwerveDriveOdometry   swerveOdometry;

 private SwerveModule frontLeft;
 private SwerveModule frontRight;
 private SwerveModule backLeft;
 private SwerveModule backRight;

private final double P = 0.009;
private final double I = 0.0;
private final double D = 0.0;
private PIDController rotatePID;


 private AHRS ahrs;

    public Drive() {
      try {
         ahrs = new AHRS(NavXComType.kMXP_SPI);
      }
      catch (RuntimeException ex) {
         System.out.println("Failed to instanciate navX");
      }

      ahrs.reset();

      while (ahrs.isConnected()== false){
         System.out.print ( ".");
      }

      while (ahrs.isCalibrating() == true) {
         System.out.print(".");
      }

      ahrs.zeroYaw();


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

        swerveOdometry = new SwerveDriveOdometry(swerveDriveKinematics,
                                                 new Rotation2d(Units.degreesToRadians(ahrs.getYaw())),
                                                 getModulePositionsMetric(),
                                                 new Pose2d( 0, 0, new Rotation2d(0)));

         rotatePID = new PIDController(P, I, D);
    
    }

   

    public void teleopDrive(double forwardPower, double strafePower, double rotatePower, boolean fieldDrive){
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(forwardPower, strafePower, rotatePower)); 
              
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);  
    }

      public SwerveModulePosition[] getModulePositionsMetric(){
         return new SwerveModulePosition[] {
            frontLeft.getModulePositionMetric(),
            frontRight.getModulePositionMetric(),
            backLeft.getModulePositionMetric(),
            backRight.getModulePositionMetric(),

         };

      }

      public Pose2d getSwervePose()  {
         return swerveOdometry.update(new Rotation2d(Units.degreesToRadians(ahrs.getYaw())),
                              getModulePositionsMetric());

      }


      // Test
    public void
     testEncoders() {
        frontLeft.testEncoder();
        frontRight.testEncoder();
        backLeft.testEncoder();
        backRight.testEncoder();
     }

     public void
     testMotors(double power) {
        frontLeft.testMotor(power);
        frontRight.testMotor(power);
        backLeft.testMotor(power);
        backRight.testMotor(power);
     }


}
