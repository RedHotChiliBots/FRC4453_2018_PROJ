package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GrabberThrow extends InstantCommand {

    public GrabberThrow() {
	super();
	requires(Robot.grabber);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
	Robot.grabber.toss();
    }

}
