package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controls {
   // XboxController controller;
    ZorroController driveController;

public static final double DRIVE_CONTROLLER_DEADZONE = 0.01;

    public Controls () {
       // controller = new XboxController(1);
        driveController = new ZorroController(0);
    }



    public double getForwardPower () {


        double power = -1 * driveController.getLeftY();


        return power;
    }

    public double getStrafePower() {

        double power = -1 * driveController.getLeftX();


        return power;
    }

    public double getRotatePower() {
        double power = driveController.getRightX();

        return power;
    }
    
}
