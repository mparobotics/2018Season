package org.usfirst.frc.team3926.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import com.ctre.CANTalon;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	
	CANTalon FR_CANTalon;
	CANTalon BR_CANTalon;
	CANTalon FL_CANTalon;
	CANTalon BL_CANTalon;
	Joystick R_stick;
	Joystick L_stick;
	
	RobotDrive driveSystem;
	
	public void robotInit() {
		
		//need to change these values to match the robot
		FR_CANTalon = new CANTalon(2);
		BR_CANTalon = new CANTalon(3);
		FL_CANTalon = new CANTalon(14);
		BL_CANTalon = new CANTalon(15);
		R_stick = new Joystick(0);
		L_stick = new Joystick(1);
		
		driveSystem = new RobotDrive(FR_CANTalon, BR_CANTalon, FL_CANTalon, BL_CANTalon);
	
	}

	
	public void teleopPeriodic() {
		
		double R_stickValue = R_stick.getY();
		double L_stickValue = L_stick.getY();
		
		if(L_stick.getRawButton(0)) {
			
			R_stickValue /= 2;
			L_stickValue /= 2;
		}
		if(R_stick.getRawButton(0)) {
			
			R_stickValue /= 2;
			L_stickValue /= 2;
		}
		
	}

}

