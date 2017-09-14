package org.usfirst.frc.team3926.robot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3926.robot.RobotMap;
import org.usfirst.frc.team3926.robot.commands.DriveCommand;

/**
 *
 */
public class DrivingSubsystem extends Subsystem {

	CANTalon FRmotor = new CANTalon(RobotMap.FRONT_RIGHT_MOTOR_ID);
	CANTalon FLmotor = new CANTalon(RobotMap.FRONT_LEFT_MOTOR_ID);
	CANTalon BRmotor = new CANTalon(RobotMap.BACK_RIGHT_MOTOR_ID);
	CANTalon BLmotor = new CANTalon(RobotMap.BACK_LEFT_MOTOR_ID);
	
	RobotDrive robotDrive = new RobotDrive(FRmotor, FLmotor, BRmotor, BLmotor);
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		
		new DriveCommand();
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
	public void TankDrive(double lSpeed, double rSpeed) {
		
		robotDrive.tankDrive(lSpeed, rSpeed);
	}
}
