package org.usfirst.frc.team4453.robot.commands.autonomous;

import org.usfirst.frc.team4453.robot.Robot;
import org.usfirst.frc.team4453.robot.Robot.RobotPosition;
import org.usfirst.frc.team4453.robot.commands.*;
import org.usfirst.frc.team4453.robot.commands.autonomous.FieldConstants.SwitchPosition;
import org.usfirst.frc.team4453.robot.subsystems.Grabber.Init;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */ 
public class ApproachAndPlaceCube2 extends CommandGroup {

    private class CenterLeft extends CommandGroup {
	public CenterLeft () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
	    addSequential(new RobotDriveDistance(12.0));
	    addSequential(new RobotTurn(-45.368));
	    addSequential(new RobotDriveDistance(82.556));
	    addSequential(new RobotTurn(45.368));
	    addSequential(new RobotDriveDistance(70.0));
	}
    }
    
    private class CenterRight extends CommandGroup {
	public CenterRight () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
	    addSequential(new RobotDriveDistance(12.0));
	    addSequential(new RobotTurn(45.368));
	    addSequential(new RobotDriveDistance(82.556));
	    addSequential(new RobotTurn(-45.368));
	    addSequential(new RobotDriveDistance(70.0));
	}
    }
    
    private class LeftLeft extends CommandGroup {
	public LeftLeft () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
	    addSequential(new RobotDriveDistance(168.0));
	    addSequential(new RobotTurn(90.0));
	    addSequential(new RobotDriveDistance(42.25));
	}
    }
    
    private class LeftRight extends CommandGroup {
	public LeftRight () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
	    addSequential(new RobotDriveDistance(210.0));
	    addSequential(new RobotTurn(90.0));
	    addSequential(new RobotDriveDistance(177.75));
	    addSequential(new RobotTurn(90.0));
	    addSequential(new RobotDriveDistance(14.0));
	}
    }
    private class RightLeft extends CommandGroup {
	public RightLeft () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.RIGHT));
	    addSequential(new RobotDriveDistance(210.0));
	    addSequential(new RobotTurn(-90.0));
	    addSequential(new RobotDriveDistance(177.75));
	    addSequential(new RobotTurn(-90.0));
	    addSequential(new RobotDriveDistance(14.0));
	}
    }
    
    private class RightRight extends CommandGroup {
	public RightRight () {
	    addSequential(new ProceedIfSwitchIsCorrect(SwitchPosition.LEFT));
	    addSequential(new RobotDriveDistance(168.0));
	    addSequential(new RobotTurn(-90.0));
	    addSequential(new RobotDriveDistance(42.25));
	}
    }
    
    public ApproachAndPlaceCube2(Robot.RobotPosition position) {
	requires(Robot.chassis);
	addSequential(Robot.grabber.new Init());
	Robot.chassis.resetNavigation();
	
	if(position == Robot.RobotPosition.LEFT) {
	    Robot.chassis.setXPos(0);
	    addParallel(new LeftLeft());
	    Robot.chassis.resetNavigation();
	    addSequential(new LeftRight());
	}
	else if(position == Robot.RobotPosition.RIGHT) {
	    Robot.chassis.setXPos(0);
	    addParallel(new RightLeft());
	    Robot.chassis.resetNavigation();
	    addSequential(new RightRight());
	}
	else if(position == Robot.RobotPosition.CENTER) {
	    Robot.chassis.setXPos(0);
	    addParallel(new CenterLeft());
	    Robot.chassis.resetNavigation();
	    addSequential(new CenterRight());
	}
	
	
//        //addParallel(new TiltForSwitch());
//        //addSequential(new RobotDriveDistance(FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH));
//       	addSequential(new GrabberTilt(130));
//	addSequential(new GrabberGrab());
//	addSequential(new RobotDriveTime(-0.75, 3.5)); // TODO - Move constants to dashboard
//	addSequential(new ProceedIfSwitchIsCorrect(position == RobotPosition.RIGHT ? SwitchPosition.RIGHT : SwitchPosition.LEFT));
//        //addSequential(new RobotDriveDistance(FieldConstants.WALL_TO_FENCE - (FieldConstants.AUTO_LINE_DIST + FieldConstants.ROBOT_LENGTH)));
//	addSequential(new GrabberTilt(20));
//	addSequential(new GrabberThrow());
//        //addSequential(new RobotDriveDistance(-FieldConstants.ROBOT_LENGTH));
    }
}
