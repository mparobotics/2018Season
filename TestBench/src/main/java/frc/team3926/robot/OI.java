package frc.team3926.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    public        Joystick rightStick;
    public        Joystick leftStick;
    public        Joystick xboxController;

    int            xboxRightAxis;
    int            xboxLeftAxis; //contols the lift

    public JoystickButton X; //controls winch motors
    public JoystickButton Y; //controls winch motors down
    public JoystickButton LB; //spits out cube (backwards)
    public JoystickButton RB; //pulls in cube (forewords)

    public JoystickButton halfSpeedTrigger; //controls half speed mode

    double         leftStickYaxis;
    double         rightStickYAxis;
    double         leftSpeed;
    double         rightSpeed;

    double         ESC;
    double         ESDB;



    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);

        xboxLeftAxis = 1;

        X = new JoystickButton(xboxController, 3);
        Y = new JoystickButton(xboxController, 4);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

        halfSpeedTrigger = new JoystickButton(rightStick, RobotMap.HALF_DRIVE_BUTTON);

        Robot.smartDashPrefs.getInstance();
        //halfSpeedTrigger.whenPressed(new DriveCommand());

    }

    public double ExponentialSpeedConstant(){

        return Robot.smartDashPrefs.getDouble("ESC",RobotMap.EXPONENTIAL_SPEED_CONSTANT);

    }

    //dead band represents the undefined area in the middle of the curve. For our purposes it will be treated as 0.
    //This is so that shakey hands to not move the robot
    public double ExponentialSpeedDeadBand(){

        return Robot.smartDashPrefs.getDouble("ESDB",RobotMap.EXPONENTIAL_SPEED_DEAD_BAND);

    }

    // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
    public double exponentialDriveRight(){

        ESC = ExponentialSpeedConstant();
        ESDB = ExponentialSpeedDeadBand();

        rightStickYAxis = Robot.oi.rightStick.getY();


        if(!(rightStickYAxis <= ESDB && rightStickYAxis >= -ESDB)) { //setting the speed to zero if in dead band

            return 0;

        } else if (rightStickYAxis < 0) { // if the joystick y axis is out of the dead band and negative

            // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
            rightSpeed = (ESC * Math.pow((-rightStickYAxis - ESDB) / (1 - ESDB), 3)) +
                         ((1 - ESC) * (rightStickYAxis + ESDB) / (1 - ESDB));

        } else { // if the joystick y axis is out of the dead band and positive

            // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
            rightSpeed = (ESC * Math.pow((rightStickYAxis - ESDB) / (1 - ESDB), 3)) +
                         ((1 - ESC) * (rightStickYAxis - ESDB) / (1 - ESDB));

        }

        //sets the speed to half the normal if the halfSpeedTrigger is pressed
        if(halfSpeedTrigger.get()) {

            rightSpeed = rightSpeed/2;

        }

        return rightSpeed;

    }

    public double exponentialDriveLeft(){

        ESC  = ExponentialSpeedConstant();
        ESDB = ExponentialSpeedDeadBand();

        leftStickYaxis = Robot.oi.leftStick.getY();

        if(!(leftStickYaxis <= ESDB && leftStickYaxis >= -ESDB)) { //setting the speed to zero if in dead band

            return 0;

        } else if (leftStickYaxis < 0) { // if the joystick y axis is out of the dead band and negative

            // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
            leftSpeed = (ESC * Math.pow((-leftStickYaxis - ESDB) / (1 - ESDB), 3)) +
                         ((1 - ESC) * (leftStickYaxis + ESDB) / (1 - ESDB));

        } else { // if the joystick y axis is out of the dead band and positive

            // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
            leftSpeed = (ESC * Math.pow((leftStickYaxis - ESDB) / (1 - ESDB), 3)) +
                         ((1 - ESC) * (leftStickYaxis - ESDB) / (1 - ESDB));

        }

        //sets the speed to half the normal if the halfSpeedTrigger is pressed
        if(halfSpeedTrigger.get()) {

            rightSpeed = rightSpeed/2;

        }

        return leftSpeed;

    }

}
