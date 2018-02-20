
package frc.team3926.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */

public class SketchyAuto extends CommandGroup {

    int posistion;
    String switchPosistion;
    double time;
    boolean scoreOnSwitchStraight;

    public SketchyAuto() {
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

        //posistion = DriverStation.getInstance().getLocation();
        switchPosistion = DriverStation.getInstance().getGameSpecificMessage();
        posistion = 2;
        time = Timer.getFPGATimestamp();

        //TODO diffrent things for diffrent posistions

        if (posistion == 1 || posistion == 3) {

            addSequential(new AutoGoStraightSides());
        } else if (posistion == 2) {

            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnRight());
            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnLeft());
            addSequential(new CenterSlam());

        }
    }
}
