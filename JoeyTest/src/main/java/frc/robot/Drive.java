package frc.robot;

public class Drive {
 public static final double SWERVE_DIST_FROM_CENTER = 0.3254375;
 public static final double Translation2d centerLocation = new Translation2d(0, 0);
 public static final double Translation2d frontLeftLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final double Translation2d frontRightLocation = new Translation2d(SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);
 public static final double Translation2d backLeftLocation = new Translation2d(-SWERVE_DIST_FROM_CENTER, SWERVE_DIST_FROM_CENTER);
 public static final double Translation2d backRightLocation = new Translation2d(-SWERVE_DIST_FROM_CENTER, -SWERVE_DIST_FROM_CENTER);
}
