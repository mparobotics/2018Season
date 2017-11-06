package org.usfirst.frc.team3926.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	RobotDrive driveSystem;
	CANTalon BL;
	CANTalon BR;
	CANTalon FL; 
	CANTalon FR;
	Joystick leftStick;
	Joystick rightStick;
	Joystick xbox; 
 	double leftStickValue;
    double rightStickValue;
 	
    public void robotInit() {
 		driveSystem = new RobotDrive(FR, BR, FL, BL);
    	
 		BL = new CANTalon(6);
 		BR = new CANTalon(5);
 		FL = new CANTalon(7);
 		FR = new CANTalon(7);
    	
 		leftStick = new Joystick(2);
 		rightStick = new Joystick(1);
 		xbox = new Joystick(0);
    }
    
    public void teleopInit() { 
    	
    		//do nothing
    }
    
    public void teleopPeriodic() {
    	
    		leftStickValue = leftStick.getY();
    		rightStickValue = rightStick.getY();
    
    		driveSystem.tankDrive(leftStickValue, rightStickValue);
    	}
}