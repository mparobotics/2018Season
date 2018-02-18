package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MiddleAutoLineCommandGroup extends CommandGroup {

    public MiddleAutoLineCommandGroup() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.

        //goes forward a bit before turning but not too much so it does not run into power cubes
        //TODO distance traveled here may need to change but it should probably stay
        addSequential(new AutonomousForwardCommand
                              ((RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET)/2));
        //turns
        addSequential(new AutonomousTurnCommand(90));
        //goes forward a bit
        addSequential(new AutonomousForwardCommand(9));//TODO set better value
        //turns to face forward
        addSequential(new AutonomousTurnCommand(-90));
        //goes forward to cross line
        addSequential
                (new AutonomousTurnCommand((RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET)/2));

    }
}