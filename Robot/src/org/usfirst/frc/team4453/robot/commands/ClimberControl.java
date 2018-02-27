package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ClimberControl extends InstantCommand {

    public ClimberControl() {
        super();
        requires(Robot.climber);
    }

    // Called once when the command executes
    protected void initialize() {
	if(Robot.oi.getDPad() >= 45 && Robot.oi.getDPad() >= 135) {
	    Robot.climber.setRightPower(Robot.oi.getClimbControl());
	}
	if(Robot.oi.getDPad() >= 225 && Robot.oi.getDPad() >= 315) {
	    Robot.climber.setLeftPower(-Robot.oi.getClimbControl());
	}
    }

}
