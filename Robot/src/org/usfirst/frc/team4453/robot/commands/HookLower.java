package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class HookLower extends InstantCommand {

    public HookLower() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	requires(Robot.hook);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
	Robot.hook.lower();
    }

}
