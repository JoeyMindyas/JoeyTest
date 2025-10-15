package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;




public class SwerveModule {
    private final double ROTATE_P = 0.007;
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;


    private SparkFlex driveMotor;
    private SparkBaseConfig driveMotorConfig;

    private SparkMax rotateMotor;
    private SparkMaxConfig rotateMotorConfig;

    private SparkAbsoluteEncoder absoluteEncoder;
    private AbsoluteEncoderConfig absoluteEncoderConfig;
    private PIDController rotatePIDController;
    
    public SwerveModule(int driveID, int rotateID, boolean invertDriveMotor){

        driveMotor       = new SparkFlex(driveID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();




        driveMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        driveMotorConfig.inverted(invertDriveMotor);

        rotateMotor         = new SparkMax(rotateID, MotorType.kBrushless);
        rotateMotorConfig   = new SparkMaxConfig();
        rotateMotorConfig.smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT);
        rotateMotorConfig.idleMode(IdleMode.kBrake);
        

        absoluteEncoder = rotateMotor.getAbsoluteEncoder();
       
        absoluteEncoderConfig = new AbsoluteEncoderConfig();
        absoluteEncoderConfig.positionConversionFactor(360);
        absoluteEncoderConfig.inverted(true);
        rotateMotorConfig.apply(absoluteEncoderConfig);


        rotatePIDController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotatePIDController.enableContinuousInput(0, 360);
        rotatePIDController.reset();


        rotateMotor.configure(rotateMotorConfig,
                             ResetMode.kNoResetSafeParameters,
                             PersistMode.kPersistParameters);

        driveMotor.configure(driveMotorConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kPersistParameters);
    }  
    
    

    public void setDesiredState(SwerveModuleState swerveModuleState) {

        double currentAngleDegrees = absoluteEncoder.getPosition();
        double targetAngleDegrees  = swerveModuleState.angle.getDegrees();
        double rotatePower         = rotatePIDController.calculate(currentAngleDegrees, targetAngleDegrees);

        driveMotor.set(MathUtil.clamp(swerveModuleState.speedMetersPerSecond, -1.0, 1.0));
        rotateMotor.set(MathUtil.clamp(rotatePower, -1.0, 1.0));
        
    }









}
