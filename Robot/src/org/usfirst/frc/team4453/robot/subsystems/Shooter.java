package org.usfirst.frc.team4453.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Shooter extends Subsystem {
    private WPI_TalonSRX   Motor;
    private DoubleSolenoid Latch;
    private DigitalInput   LimitSwitch;
    private Relay	   Clutch;

    public void fire() {
	Latch.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {

    }

    public void prepare() {
	Clutch.set(Relay.Value.kOn);
	Motor.set(1);
	while (!LimitSwitch.get()) {
	    try {
		Thread.sleep(10);
	    }
	    catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	    }
	}
	Motor.set(0);
	Latch.set(DoubleSolenoid.Value.kForward);
	try {
	    Thread.sleep(500);
	}
	catch (InterruptedException e) {
	    // TODO Auto-generated catch block
	    e.printStackTrace();
	}
	Clutch.set(Relay.Value.kOff);
    }
}
