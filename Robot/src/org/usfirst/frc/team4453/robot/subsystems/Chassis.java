package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 *
 */
public class Chassis extends Subsystem {
	private WPI_TalonSRX leftFront  = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR), 
    		 	         rightFront = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR),
    		 	         leftBack   = new WPI_TalonSRX(RobotMap.REAR_LEFT_MOTOR),
    		 	         rightBack  = new WPI_TalonSRX(RobotMap.REAR_RIGHT_MOTOR);
	
	private MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void drive(double xspeed, double yspeed, double rotation)
    {
    	drive.driveCartesian(yspeed, xspeed, rotation);
    }
    
    public void stop()
    {
    	drive.stopMotor();
    }
}

