package frc.team3926.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {

    private Joystick rightStick;
    private Joystick leftStick;
    public Joystick xboxController;

    int xboxRightAxis;
    int xboxLeftAxis; //contols the lift

    public JoystickButton X; //controls winch motors
    public JoystickButton Y; //controls winch motors down
    public JoystickButton LB; //spits out cube (backwards)
    public JoystickButton RB; //pulls in cube (forewords)

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

        xboxLeftAxis = 1;
        xboxRightAxis = 5;

        X = new JoystickButton(xboxController, 3);
        Y = new JoystickButton(xboxController, 4);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

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
    }

    public final double getJoystickLeftY() {

        double rawY = leftStick.getY();
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN, RobotMap.OI_DEAD_BAND);
    }

    public final double getJoystickRightY() {

        double rawY = rightStick.getY();
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_GAIN, RobotMap.OI_DEAD_BAND);
    }

    public final double getXboxLeftY() {

        double rawY = xboxController.getRawAxis(xboxLeftAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_DEAD_BAND);
    }

    public final double getXboxRightY() {

        double rawY = xboxController.getRawAxis(xboxRightAxis);
        return apply_gain_deadzone_exponential(rawY, RobotMap.OI_XBOX_GAIN, RobotMap.OI_DEAD_BAND);
    }

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
