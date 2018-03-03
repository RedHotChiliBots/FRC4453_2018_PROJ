package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.Robot.RobotPosition;
import org.usfirst.frc.team4453.robot.commands.*;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ApproachAndPlaceCube extends CommandGroup {

    public ApproachAndPlaceCube(Robot.RobotPosition position) {
        //addParallel(new TiltForSwitch());
        //addSequential(new RobotDriveDistance(FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH));
        
	addSequential(new WaitForGrabberInit());
	addSequential(new GrabberTilt(155));
	addSequential(new GrabberGrab());
	addSequential(new RobotDriveTime(-0.75, 3.5)); // TODO
	addSequential(new ProceedIfSwitchIsCorrect(position == RobotPosition.RIGHT ? SwitchPosition.RIGHT : SwitchPosition.LEFT));
        //addSequential(new RobotDriveDistance(FieldConstants.WALL_TO_FENCE - (FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH)));
	addSequential(new GrabberTilt(20));
	addSequential(new GrabberThrow());
        //addSequential(new RobotDriveDistance(-FieldConstants.ROBOT_LENGTH));
    }
}
