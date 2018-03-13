package org.usfirst.frc.team4453.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoTest extends CommandGroup {

    public AutoTest() {
	addSequential(new RobotTurn(-90));
	addSequential(new RobotTurn(90));
	//addSequential(new RobotDriveTime(0.5, 3));
    }
}
