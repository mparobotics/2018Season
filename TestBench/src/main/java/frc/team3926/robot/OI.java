package frc.team3926.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    public        Joystick rightStick;
    public        Joystick leftStick;
    public        Joystick xboxController;

    int            xboxRightAxis;
    int            xboxLeftAxis; //contols the lift

    double         leftStickYaxis;
    double         rightStickYaxis;
    double         leftSpeed;
    double         rightSpeed;

    double         ESC;
    double         ESP;



    JoystickButton X; //controls winch motor
    JoystickButton LB; //controls left side of intake
    JoystickButton RB; //controls right side of intake

    OI() {

        rightStick = new Joystick(RobotMap.RIGHT_JOYSTICK);
        leftStick = new Joystick(RobotMap.LEFT_JOYSTICK);
        xboxController = new Joystick(RobotMap.XBOX_CONTROLLER);

        xboxLeftAxis = 2;

        X = new JoystickButton(xboxController, 3);
        LB = new JoystickButton(xboxController, 5);
        RB = new JoystickButton(xboxController, 6);

        Robot.smartDashPrefs.getInstance();

    }

    public double ExponentialSpeedConstant(){

        return Robot.smartDashPrefs.getDouble("ESC",0);

    }

    public double ExponentialSpeedPower(){

        return Robot.smartDashPrefs.getDouble("ESP",3);

    }

    public double exponentialDriveRight(){

        ESC = ExponentialSpeedConstant();
        ESP = ExponentialSpeedPower();

        rightStickYaxis = Robot.oi.rightStick.getY();

        // plugs the y axis of the right joystick into a cubic equation, resulting in the speed
        rightSpeed = ESC * (Math.pow(rightStickYaxis, ESP))
                     + (1 - ESC) * rightStickYaxis;

        return rightSpeed;

    }

    public double exponentialDriveLeft(){

        ESC = ExponentialSpeedConstant();
        ESP = ExponentialSpeedPower();

        leftStickYaxis = Robot.oi.leftStick.getY();

        // plugs the y axis of the left joystick into a cubic equation, resulting in the speed
        leftSpeed = ESC * (Math.pow(leftStickYaxis, ESP))
                    + (1 - ESC) * leftStickYaxis;

        return leftSpeed;

    }

}
