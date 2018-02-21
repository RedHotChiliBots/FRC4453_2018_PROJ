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

    // ====== Constants ======
    public static final DoubleSolenoid.Value SHIFTER_HIGH_GEAR	       = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value SHIFTER_LOW_GEAR	       = DoubleSolenoid.Value.kReverse;

    public static final boolean		     HIGH_GEAR		       = true;
    public static final boolean		     LOW_GEAR		       = false;

    // ====== Motor Controllers ======
    // Chassis
    public static final int		     CHASSIS_FRONT_RIGHT_MOTOR = 0;			       // TODO
    public static final int		     CHASSIS_FRONT_LEFT_MOTOR  = 1;			       // TODO
    public static final int		     CHASSIS_MID_RIGHT_MOTOR   = 2;			       // TODO
    public static final int		     CHASSIS_MID_LEFT_MOTOR    = 3;			       // TODO
    public static final int		     CHASSIS_REAR_RIGHT_MOTOR  = 4;			       // TODO
    public static final int		     CHASSIS_REAR_LEFT_MOTOR   = 5;			       // TODO
    // Climber
    public static final int		     CLIMBER_MOTOR_RIGHT       = 6;			       // TODO
    public static final int		     CLIMBER_LEFT_MOTOR	       = 7;			       // TODO
    public static final int		     CLIMBER_HOOK_MOTOR	       = 12;
    // Grabber
    public static final int		     GRABBER_LEFT_MOTOR	       = 8;			       // TODO
    public static final int		     GRABBER_RIGHT_MOTOR       = 9;			       // TODO
    public static final int		     GRABBER_TILT_MOTOR	       = 10;			       // TODO
    // Shooter
    public static final int		     SHOOTER_WIND_MOTOR	       = 11;
    //

    // ====== Solenoids ======
    // Chassis
    public static final int		     SHIFTER_HI_SOLENOID       = 0;			       // TODO
    public static final int		     SHIFTER_LO_SOLENOID       = 1;			       // TODO
    // Lifter
    public static final int		     LIFTER_UP_SOLENOID	       = 2;			       // TODO
    public static final int		     LIFTER_DOWN_SOLENOID      = 3;			       // TODO
    // Grabber
    public static final int		     GRABBER_GRIP_SOLENOID     = 4;			       // TODO
    public static final int		     GRABBER_RELEASE_SOLENOID  = 5;			       // TODO
    // Shooter
    public static final int		     SHOOTER_LATCH_SOLENOID    = 6;			       // TODO
    public static final int		     SHOOTER_RELEASE_SOLENOID  = 7;			       // TODO

    // ====== Analog Inputs ======
    // Pressure Sensor
    public static final int		     HI_PRESSURE_SENSOR	       = 0;
    public static final int		     LO_PRESSURE_SENSOR	       = 1;
    // Distance Sensor
    public static final int		     LEFT_DISTANCE_SENSOR      = 2;
    public static final int		     RIGHT_DISTANCE_SENSOR     = 3;

    // ====== Digital Inputs ======
    public static final int		     SHOOTER_LIMIT_SWITCH      = 0;

    // ====== Relays ======
    public static final int		     SHOOTER_CLUTCH_RELAY      = 0;

}
