package frc.team3926.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3926.robot.Robot;

import static frc.team3926.robot.Robot.driveSubsystem;

/**
 *
 */
public class MoreAuto extends Command {

    public MoreAuto() {

        requires(driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.sensorSubsystem.setClosedEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.sensorSubsystem.getRobotPosistion();
        SmartDashboard.putNumber("Robot Sensor X: ", Robot.sensorSubsystem.X);
        SmartDashboard.putNumber("Robot Sensor Y: ", Robot.sensorSubsystem.Y);
        SmartDashboard.putNumber("Robot Gyro Angle: ", Robot.sensorSubsystem.gyroAngle);

        Robot.driveSubsystem.headingControl(60, -60, (Math.PI / 2), true, Robot.sensorSubsystem.X,
                                            Robot.sensorSubsystem.Y, Robot.sensorSubsystem.gyroAngle);

        driveSubsystem.setSpeed(driveSubsystem.leftPI(driveSubsystem.leftVelocity),
                                driveSubsystem.rightPI(driveSubsystem.rightVelocity));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return false;
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}
