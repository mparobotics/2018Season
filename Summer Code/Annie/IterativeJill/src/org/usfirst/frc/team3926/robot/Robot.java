package org.usfirst.frc.team3926.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	
	public final static int FRONT_RIGHT_MOTOR_ID = 13; //inputs for Jill (might need to change)
    public final static int FRONT_LEFT_MOTOR_ID  = 12;
    public final static int BACK_RIGHT_MOTOR_ID  = 15;
    public final static int BACK_LEFT_MOTOR_ID   = 1;
    
    public final static int RIGHT_STICK_ID       = 0;
    public final static int LEFT_STICK_ID        = 1;
    
    CANTalon FR_CAN;
    CANTalon BR_CAN;
    CANTalon FL_CAN;
    CANTalon BL_CAN;
    
    Joystick rStick;
    Joystick lStick;
    
    public RobotDrive driveSystem;
	
    double leftInput;
	double rightInput;
	
	public void robotInit() {
		
		FR_CAN      = new CANTalon(FRONT_RIGHT_MOTOR_ID);
		BR_CAN      = new CANTalon(BACK_RIGHT_MOTOR_ID);
		FL_CAN      = new CANTalon(FRONT_LEFT_MOTOR_ID);
		BL_CAN      = new CANTalon(BACK_LEFT_MOTOR_ID);
		
		rStick      = new Joystick(RIGHT_STICK_ID);
		lStick      = new Joystick(LEFT_STICK_ID);
		
		driveSystem = new RobotDrive(FR_CAN, BR_CAN, FL_CAN, BL_CAN);
	}

	public void teleopPeriodic() {
		
		leftInput  = lStick.getY();
		rightInput = rStick.getY();
		
		driveSystem.tankDrive(leftInput, rightInput);
	}

	
}

