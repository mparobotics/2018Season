package frc.team3926.robot;

public class RobotMap {
////////////////////////////////test sensors, motors, and values////////////////////////////////////////
    public static int LIMIT_SWITCH = 0;
    public static int ENCODER_ID_1 = 1;
    public static int ENCODER_ID_2 = 2;

    public static double RADIUS_OF_ROTATION = 7.62; //cm
    public static int ENCODER_MOTOR = 2; //testing motor

///////////////////////////////drive train motors///////////////////////////////////////////////////////
    public static int FRONT_RIGHT = 8;  //(BMO)
    public static int FRONT_LEFT = 22;
    public static int BACK_RIGHT = 7;
    public static int BACK_LEFT = 0;

///////////////////////////////intake, winch, and lift motors and values////////////////////////////////
    public static int INTAKE_MOTOR_1 = 1; //change id, left,   25% to 30% speed
    public static int INTAKE_MOTOR_2 = 2; //change id, right,  25% to 30% speed
    public static double INTAKE_SPEED = .30;

    public static int WINCH_MOTOR_1 = 3; //change id,      90% speed
    public static int WINCH_MOTOR_2 = 4; //change id,      90% speed
    public static double WINCH_SPEED = .90;

    public static int LIFT_MOTOR = 5; //change id, adjustable up to 75% speed
    public static int UP_LIFT_LIMIT_SWITCH = 4; //change id
    public static int DOWN_LIFT_LIMIT_SWITCH = 2; //change id

//////////////////////////////joysticks/////////////////////////////////////////////////////////////////
    public static int RIGHT_JOYSTICK = 0;
    public static int LEFT_JOYSTICK = 1;
    public static int XBOX_CONTROLLER = 3;

    public static int HALF_DRIVE_BUTTON = 1;

/////////////////////////////camera////////////////////////////////////////////////////////////////////
    public static int CAMERA_RES_HIGHT = 480;
    public static int CAMERA_RES_WIDTH = 640;
    public static int FPS = 20;
////////////////////////////other/////////////////////////////////////////////////////////////////////
    public static double EXPONENTIAL_SPEED_CONSTANT = .5; //TODO find better value


}
