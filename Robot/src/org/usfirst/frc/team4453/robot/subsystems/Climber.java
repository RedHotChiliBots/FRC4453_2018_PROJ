package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	private static final double CLIMB_SPEED = 1.0; //TODO
   
	private WPI_TalonSRX climber = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void climb()
    {
    	climber.set(CLIMB_SPEED);
    }
    
    public void stop()
    {
    	climber.set(0);
    }
}

