package org.usfirst.frc.team4453.robot.commands;

import org.usfirst.frc.team4453.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class HookTeleop extends InstantCommand {

    private static final double SCALE = 0.1;
    
    public HookTeleop() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
	requires(Robot.hook);
    }

    // Called just before this Command runs the first time
    @Override
    protected void execute() {
	Robot.hook.set(Robot.hook.getDistance() + Robot.oi.getHookSpeed() * SCALE);
    }

}
