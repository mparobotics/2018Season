package frc.team3926.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCommandGroup extends CommandGroup {

    public AutonomousCommandGroup() {
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

        switch(RobotMap.STARTING_POSITION){

            case "right":
                // crosses auto line
                addSequential
                        (new AutonomousForwardCommand(RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET));
                // add more commands to go through the complete routine. For now this will be used to test
                break;

            case "middle":
                // add more commands to go through the complete routine. For now this will be used to test
                break;

            case "left":
                // crosses auto line
                addSequential
                        (new AutonomousForwardCommand(RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET));
                // add more commands to go through the complete routine. For now this will be used to test
                break;

        }
        //crosses start line if not in middle


    }
}