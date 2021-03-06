package frc.team3926.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {

    private Joystick rightStick;
    private Joystick leftStick;
    public Joystick xboxController;
    public Joystick xboxDriveController;

    int xboxRightAxis;
    int xboxLeftAxis; //contols the lift

    public JoystickButton X; //controls winch motors
    public JoystickButton Y; //controls winch motors down
    public JoystickButton A;
    public JoystickButton B;
    public JoystickButton BACK;
    public JoystickButton LB; //spits out cube (backwards)
    public JoystickButton RB; //pulls in cube (forewords)
    public JoystickButton xboxButton; //releases the wings

    public JoystickButton halfSpeedTrigger; //controls half speed mode
    public JoystickButton straightModeTrigger; //controls straight mode

    public SendableChooser gainChooser;
    public SendableChooser deadBandChooser;

    //private double gain = 0;
    //private double deadBand = 0;

    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);
        //xboxDriveController = new Joystick(RobotMap.XBOX_DRIVE_CONTROLLER);

        xboxLeftAxis = 1;
        xboxRightAxis = 5;

        X = new JoystickButton(xboxController, 3);
        Y = new JoystickButton(xboxController, 4);
        A = new JoystickButton(xboxController, 1);
        B = new JoystickButton(xboxController, 2);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);
        BACK = new JoystickButton(xboxController, 7);

        halfSpeedTrigger = new JoystickButton(rightStick, 1);
        straightModeTrigger = new JoystickButton(leftStick, 1);

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

       //X.whenPressed(Robot.StraightenCube);
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

        double rawY = leftStick.getY();

        SmartDashboard.putNumber("Raw Y left: ", rawY);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN_R, RobotMap.OI_DEAD_BAND_R);
        //return rawY;
    }

    public final double getJoystickRightY() {

        double rawY = rightStick.getY();

        SmartDashboard.putNumber("Raw Y right: ", rawY);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN_R, RobotMap.OI_DEAD_BAND_R);
        //return rawY;
    }

    public final double getXboxLeftY() {

        double rawY = xboxController.getRawAxis(xboxLeftAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
    }

    public final double getXboxRightY() {

        double rawY = xboxController.getRawAxis(xboxRightAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
    }

    /*public final double getDriveXboxLeftY() {

        double rawY = xboxDriveController.getRawAxis(xboxLeftAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
    }

    public final double getDriveXboxRightY() {

        double rawY = xboxDriveController.getRawAxis(xboxRightAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_XBOX_DEAD_BAND);
    }*/

    private double apply_gain_deadzone_exponential(double rawY, double gain, double deadBand) {

        double absY = Math.abs(rawY);
        /*gain = SmartDashboard.getNumber("Gain", gain);
        deadBand = SmartDashboard.getNumber("Dead Band", deadBand);*/

        SmartDashboard.putNumber("Gain:",  gain);
        SmartDashboard.putNumber("Dead Band:", deadBand);
        SmartDashboard.putNumber("Raw Y:", rawY);

        if(absY <= deadBand) {

            return 0;

        } else {

             absY = (gain * Math.pow((absY - deadBand) / (1 - deadBand), 3)) +
                         ((1 - gain) * (absY - deadBand) / (1 - deadBand));
        }

        if(rawY < 0) {

            return -absY;
        } else {

            return absY;
        }
    }

}
