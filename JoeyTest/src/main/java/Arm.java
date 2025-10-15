import edu.wpi.first.math.controller.PIDController;

public class Arm {

    private final double LOWER_P = 0.003;
    private final double LOWER_I = 0;
    private final double LOWER_D = 0;
    private final double LOWER_TOL = 2; //degrees


    private final double UPPER_P = 0.003;
    private final double UPPER_I = 0;
    private final double UPPER_D = 0;
    private final double UPPER_TOL = 2; //degrees
   
    PIDController lowerPidController;
    
    PIDController upperPidController;


    public Arm()  {

        lowerPidController = new PIDController(LOWER_P, LOWER_I, LOWER_D);
        lowerPidController.setTolerance(LOWER_TOL);
        lowerPidController.reset();



        upperPidController = new PIDController(UPPER_P, UPPER_I, UPPER_D);
        upperPidController.setTolerance(UPPER_TOL);
        upperPidController.reset();


    
    }





}
