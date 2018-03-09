package frc.team3926.robot;

public class RobotMap {

    public static boolean BMO = false;
    public static boolean QBERT = true;
////////////////////////////////test sensors, motors, and values////////////////////////////////////////
    public static int LIMIT_SWITCH = 0;
    public static int ENCODER_ID_1 = 1;
    public static int ENCODER_ID_2 = 2;

    public static double RADIUS_OF_ROTATION = 7.62; //cm
    public static int ENCODER_MOTOR = 2; //testing motor

///////////////////////////////drive train motors///////////////////////////////////////////////////////
    public static int FRONT_RIGHT = 2;  //(Qbert)
    public static int FRONT_LEFT = 6;
    public static int BACK_RIGHT = 5;
    public static int BACK_LEFT = 4;

    public static int BMO_FRONT_RIGHT = 8; // (BMO)
    public static int BMO_FRONT_LEFT = 22;
    public static int BMO_BACK_RIGHT = 7;
    public static int BMO_BACK_LEFT = 2;

///////////////////////////////intake, winch, and lift motors and values////////////////////////////////
    public static int INTAKE_MOTOR_1 = 3; //change id, left,   25% to 30% speed
    public static int INTAKE_MOTOR_2 = 7; //change id, right,  25% to 30% speed
    public static double INTAKE_SPEED = .30;

    //public static int WINCH_MOTOR_1 = 8; //change id,      90% speed
    public static int CLIMBING_WINCH_MOTOR_2 = 9; //change id,      90% speed
    public static double WINCH_SPEED = .90; //changed which speed from 90% to 40%

    public static int LIFT_MOTOR = 8; //change id, adjustable up to 75% speed
    public static int UP_LIFT_LIMIT_SWITCH = 4; // TODO change id
    public static int DOWN_LIFT_LIMIT_SWITCH = 2; // TODO change id

//////////////////////////////joysticks/////////////////////////////////////////////////////////////////
    public static int RIGHT_JOYSTICK = 0;
    public static int LEFT_JOYSTICK = 1;
    public static int XBOX_CONTROLLER = 2;

/////////////////////////////camera////////////////////////////////////////////////////////////////////
    public static int CAMERA_RES_HEIGHT = 240;
    public static int CAMERA_RES_WIDTH = 320;
    public static int FPS = 30;
////////////////////////////other/////////////////////////////////////////////////////////////////////
    public static double OI_GAIN_R = .3;
    public static double OI_DEAD_BAND_R = .1;

    public static double OI_GAIN_L = .3;
    public static double OI_DEAD_BAND_L = .025;


    public static double OI_XBOX_GAIN = .05;
    public static double OI_XBOX_DEAD_BAND = .1;

    public static int WING_SERVO_ID = 0; //TODO change


    public static int DRIVE_MODE_STRAIGHT = 3;
    public static int DRIVE_MODE_HALF = 2;
    public static int DRIVE_MODE_DEFAULT = 1;

}
