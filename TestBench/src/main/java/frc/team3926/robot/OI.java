package frc.team3926.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class OI {

    public Joystick rightStick;
    public Joystick leftStick;
    public Joystick xboxController;

    int            xboxRightAxis;
    int            xboxLeftAxis; //contols the lift

    public JoystickButton X; //controls winch motors
    public JoystickButton Y; //controls winch motors down
    public JoystickButton LB; //spits out cube (backwards)
    public JoystickButton RB; //pulls in cube (forewords)
    public JoystickButton xboxButton; //releases the wings

    public JoystickButton halfSpeedTrigger; //controls half speed mode
    public JoystickButton straightModeTrigger; //controls straight mode

    public SendableChooser gainChooser;
    public SendableChooser deadBandChooser;

    //private double gain = 0;
    //private double deadBand = 0;

    double         leftStickYaxis;
    double         rightStickYAxis;
    double         leftSpeed;
    double         rightSpeed;

    double         ESC;
    double         ESDB;
    Preferences sensorSmartdashPrefs;



    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);

        xboxLeftAxis = 1;
        xboxRightAxis = 5;

        xboxButton = new JoystickButton(xboxController, 7);
        X = new JoystickButton(xboxController, 3);
        Y = new JoystickButton(xboxController, 4);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

        halfSpeedTrigger = new JoystickButton(rightStick, RobotMap.HALF_DRIVE_BUTTON);

        sensorSmartdashPrefs = Preferences.getInstance();
        //halfSpeedTrigger.whenPressed(new DriveCommand());

        /*gainChooser = new SendableChooser();
        deadBandChooser = new SendableChooser();

        gainChooser.addDefault("Gain", gain);
        deadBandChooser.addDefault("Dead Band", deadBand);*/

        //SmartDashboard.putData("Gain", gainChooser);
        //SmartDashboard.putData("Dead Band", deadBandChooser);

        //gain = (double) gainChooser.getSelected();
        //deadBand = (double) deadBandChooser.getSelected();

       /* SmartDashboard.putNumber("Gain", gain);
        SmartDashboard.putNumber("Dead Band", deadBand);*/
    }

    public final void setXboxRumble(boolean rumbleOn) {

        if(rumbleOn) {

            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
        } else {

            xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    public final double getJoystickLeftY() {

        double output;
        double rawY = leftStick.getY();
        output = rawY;
        //output = apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN, RobotMap.OI_DEAD_BAND);
        return output;
    }

    public final double getJoystickRightY() {

        double output;
        double rawY = rightStick.getY();
        output = rawY;
        //output = apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN, RobotMap.OI_DEAD_BAND);
        return output;
    }

    public final double getXboxLeftY() {

        double output;
        double rawY = xboxController.getRawAxis(xboxLeftAxis);
        output = rawY;
        //output = apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
        return output;
    }

    public final double getXboxRightY() {

        double output;
        double rawY = xboxController.getRawAxis(xboxRightAxis);
        output = rawY;
        //output = apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
        return output;

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

    public double exponentialDriveLeft() {

        ESC = ExponentialSpeedConstant();
        ESDB = ExponentialSpeedDeadBand();

        leftStickYaxis = Robot.oi.leftStick.getY();

        if (!(leftStickYaxis <= ESDB && leftStickYaxis >= -ESDB)) { //setting the speed to zero if in dead band

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
        if (halfSpeedTrigger.get()) {

            leftSpeed = leftSpeed / 2;

        }
        return leftSpeed;
    }



}
