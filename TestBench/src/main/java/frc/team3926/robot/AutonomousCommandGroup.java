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
                //TODO distance traveled here may need to change but it should probably stay
                addSequential
                        (new AutonomousForwardCommand(RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET +1));
                // add more commands to go through the complete routine. For now this will be used to test
                break;

            case "middle":

                //goes forward a bit, goes to one of the sides and then finally crosses the auto line.
                addSequential(new MiddleAutoLineCommandGroup());

                // add more commands to go through the complete routine. For now this will be used to test
                break;

            case "left":
                // crosses auto line
                //TODO distance traveled here may need to change but it should probably stay
                addSequential
                        (new AutonomousForwardCommand(RobotMap.AUTO_LINE_DISTANCE_FEET - RobotMap.ROBOT_LENGTH_FEET));
                // add more commands to go through the complete routine. For now this will be used to test
                break;

        }

        // goes forward to line up with switch
        addSequential(new AutonomousForwardCommand(RobotMap.AUTO_LINE_TO_SWITCH_FEET));

        // turns toward switch depending on side the robot is on and then puts the power cube in the switch. then turns
        // forward
        switch(RobotMap.STARTING_POSITION){

            // assumes that if the robot starts in the middle it has gone to the right side.
            case "middle":
            case "right":
                //turns to face towards switch
                addSequential(new AutonomousTurnCommand(-90));
                //TODO add command to put power cube in switch
                //turns to face forward
                addSequential(new AutonomousTurnCommand(90));
                break;

            case "left":
                //turns to face towards switch
                addSequential(new AutonomousTurnCommand(90));
                //TODO add command to put power cube in switch
                //turns to face forward
                addSequential(new AutonomousTurnCommand(-90));
                break;

        }

        //goes forward to clear way for other robots
        addSequential(new AutonomousForwardCommand(8)); //TODO set better distance

    }

}