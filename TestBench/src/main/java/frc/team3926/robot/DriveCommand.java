
package frc.team3926.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class DriveCommand extends Command {

    public Preferences dashboardPreferences;
    public boolean safeMode = true;

    public DriveCommand() {

        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time

    @Override
    protected void initialize() {

       /* dashboardPreferences = Preferences.getInstance();
        dashboardPreferences.putBoolean("Safe Mode: ", safeMode);
        SmartDashboard.putBoolean("Safe Mode: ", safeMode);*/

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        //SmartDashboard.putNumber("Left RPM: ", Robot.driveSubsystem.getLeftRPM());
        //SmartDashboard.putNumber("Right RPM: ", Robot.driveSubsystem.getRightRPM());
        //safeMode = dashboardPreferences.getBoolean("Safe Mode: ", safeMode);

        SmartDashboard.putBoolean("half speed trigger: ", Robot.oi.halfSpeedTrigger.get());

        Robot.driveSubsystem.teleopDrive();

        if(RobotMap.QBERT) {

            Robot.climbingSubsystem.teleopClimb();
            Robot.intakeArmSubsystem.teleopIntake();
            Robot.liftSubsystem.controlLiftTeleop();
        }

        /*if (Robot.sensorSubsystem.LimitSwitch()) {
            Robot.oi.xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, .5);
            Robot.oi.xboxController.setRumble(GenericHID.RumbleType.kRightRumble, .5);
            Timer.delay(.3);

            Robot.oi.xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            Robot.oi.xboxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }*/


    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {

        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {

    }

}

