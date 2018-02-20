
package frc.team3926.robot.command.auto;

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

        //sides
        if(posistion == 1 && switchPosistion.startsWith("l")) {

            addSequential(new frc.team3926.robot.command.auto.AutoGoStraightSides()); //score on switch

        } else if (posistion == 1 && switchPosistion.startsWith("r")) {

            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle()); //dont score on switch (turn left)

            addSequential(new AutoTurnLeft());
            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnRight());
            addSequential(new CenterSlam());
        }
        if(posistion == 3 && switchPosistion.startsWith("r")) {

            addSequential(new AutoGoStraightSides()); //score on switch

        } else if (posistion == 3 && switchPosistion.startsWith("l")) {

            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle()); //dont score on switch (turn right)

            addSequential(new frc.team3926.robot.command.auto.AutoTurnRight());
            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle());

            addSequential(new frc.team3926.robot.command.auto.AutoTurnLeft());
            addSequential(new frc.team3926.robot.command.auto.CenterSlam());
        }

        //center
        if(posistion == 2 && switchPosistion.startsWith("r")) {

            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle()); //turn right to score

            addSequential(new frc.team3926.robot.command.auto.AutoTurnRight());
            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle());

            addSequential(new frc.team3926.robot.command.auto.AutoTurnLeft());
            addSequential(new frc.team3926.robot.command.auto.CenterSlam());

        } else if(posistion == 2 && switchPosistion.startsWith("l")) {

            addSequential(new frc.team3926.robot.command.auto.AutoStraightMiddle());

            addSequential(new AutoTurnLeft());
            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnRight());
            addSequential(new CenterSlam());
        }


        /*if (posistion == 1 || posistion == 3) {

            addSequential(new AutoGoStraightSides());
        } else if (posistion == 2) {

            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnRight());
            addSequential(new AutoStraightMiddle());

            addSequential(new AutoTurnLeft());
            addSequential(new CenterSlam());

        }*/
    }
}
