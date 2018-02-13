/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4453.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // Chassis
    // Motor Controllers
    public static final int		     CHASSIS_FRONT_RIGHT_MOTOR	   = 0;				   // TODO
    public static final int		     CHASSIS_FRONT_LEFT_MOTOR	   = 1;				   // TODO
    public static final int		     CHASSIS_MID_RIGHT_MOTOR	   = 2;				   // TODO
    public static final int		     CHASSIS_MID_LEFT_MOTOR	   = 3;				   // TODO
    public static final int		     CHASSIS_REAR_RIGHT_MOTOR	   = 4;				   // TODO
    public static final int		     CHASSIS_REAR_LEFT_MOTOR	   = 5;				   // TODO
    // Solenoids
    public static final int		     SHIFTER_FWD_SOLENOID	   = 0;				   // TODO
    public static final int		     SHIFTER_REV_SOLENOID	   = 1;				   // TODO
    // Constants
    public static final DoubleSolenoid.Value SHIFTER_HIGH_GEAR		   = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value SHIFTER_LOW_GEAR		   = DoubleSolenoid.Value.kReverse;

    // Climber
    // Motor Controllers
    public static final int		     CLIMBER_MOTOR_RIGHT	   = 6;				   // TODO
    public static final int		     CLIMBER_LEFT_MOTOR		   = 7;				   // TODO

    // Lifter
    // Solenoids
    public static final int		     SOLENOID_LIFTER_LEFT	   = 2;				   // TODO
    public static final int		     SOLENOID_LIFTER_RIGHT	   = 3;				   // TODO

    // Grabber
    // Motor Controllers
    public static final int		     GRABBER_LEFT_MOTOR		   = 8;				   // TODO
    public static final int		     GRABBER_RIGHT_MOTOR	   = 9;				   // TODO
    public static final int		     GRABBER_TILT_MOTOR		   = 10;			   // TODO
    // Solenoids
    public static final int		     GRABBER_GRIP_SOLENOID_GRIP	   = 4;				   // TODO
    public static final int		     GRABBER_GRIP_SOLENOID_RELEASE = 5;				   // TODO

    // Analog Inputs
    public static final int		     HI_PRESSURE_SENSOR		   = 0;
    public static final int		     LO_PRESSURE_SENSOR		   = 1;
}
