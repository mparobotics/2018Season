package frc.team3926.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;

/**
 *
 */
public class ControlLiftCommand extends Command {

    public ControlLiftCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

        requires(Robot.liftSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.liftSubsystem.setLiftSpeed(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Robot.liftSubsystem.controlLiftTeleop();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        Boolean limitSwitchIsHit = false;

        if(Robot.sensorSubsystem.HeightLimit() || Robot.sensorSubsystem.DownLimit()) {

            limitSwitchIsHit = true;

            Robot.oi.xboxController.setRumble(kLeftRumble, 1);
            Robot.oi.xboxController.setRumble(kRightRumble, 1);
            Timer.delay(.075);
            Robot.oi.xboxController.setRumble(kLeftRumble, 0);
            Robot.oi.xboxController.setRumble(kRightRumble, 0);
        }

        return limitSwitchIsHit;
    }

    // Called once after isFinished returns true
    protected void end() {

        Robot.liftSubsystem.setLiftSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }

}