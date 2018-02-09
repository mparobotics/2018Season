package frc.team3926.robot;

public class RobotMap {
////////////////////////////////test sensors, motors, and values////////////////////////////////////////
    public static int LIMIT_SWITCH = 0;

    public static double RADIUS_OF_ROTATION = 7.62; //cm

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


    public static double AUTONOMOUS_SPEED_TOLERANCE  = 1; //tbd

    public static double AUTO_LINE_DISTANCE_METERS   = 1; //tbd
    public static String STARTING_POSITION           = "right"; //tbd
    public static String FREINDLY_SCALE_SIDE         = "right"; //tbd

    public static int    FORWARD_KP                  = 1; //tbd
    public static int    FORWARD_KI                  = 1; //tbd
    public static int    FORWARD_KD                  = 1; //tbd
    public static int    FORWARD_SPEED_SETPOINT      = 1; //tbd

    public static int TURNING_KP                     = 1; //tbd
    public static int TURNING_KI                     = 1; //tbd
    public static int TURNING_KD                     = 1; //tbd
    public static int TURNING_SPEED_SETPOINT         = 1; //tbd

    public static int LEFT_DRIVE_ENC_PORT_ONE        = 1; //tbd
    public static int LEFT_DRIVE_ENC_PORT_TWO        = 1; //tbd
    public static int RIGHT_DRIVE_ENC_PORT_ONE       = 1; //tbd
    public static int RIGHT_DRIVE_ENC_PORT_TWO       = 1; //tbd

    public static double STARTING_X_POSITION         = 0; //tbd
    public static double STARTING_Y_POSITION         = 0; //tbd

}
