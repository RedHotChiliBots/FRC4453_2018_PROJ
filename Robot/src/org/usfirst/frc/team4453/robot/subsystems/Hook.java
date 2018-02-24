package org.usfirst.frc.team4453.robot.subsystems;

import org.usfirst.frc.team4453.robot.RobotMap;
import org.usfirst.frc.team4453.robot.commands.HookStop;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Hook extends Subsystem {

    // PID Constants
    public final int	 slotIdx     = 0;
    public final int	 pidLoopIdx  = 0;
    public final int	 timeOutMs   = 10;
    public final boolean sensorPhase = true;
    public final boolean motorInvert = false;
    public final double	 kF	     = 0.0;
    public final double	 kP	     = 1.0;
    public final double	 kI	     = 0.0;
    public final double	 kD	     = 0.0;

    // Encoder Resolution
    //
    public final int	COUNTS_PER_REV_MOTOR   = 48;	// 12 cpr quadrature - RS7 Encoder from Armabot
    public final int	GEARBOX_RATIO	       = 20;
    public final double	SHAFT_DIAMETER	       = 1.0;
    public final int	COUNTS_PER_REV_GEARBOX = COUNTS_PER_REV_MOTOR * GEARBOX_RATIO;
    public final double	INCHES_PER_REV	       = SHAFT_DIAMETER * Math.PI;
    public final double	COUNTS_PER_INCH	       = COUNTS_PER_REV_GEARBOX / INCHES_PER_REV;

    // Motor
    private final WPI_TalonSRX hookLift = new WPI_TalonSRX(RobotMap.CLIMBER_HOOK_MOTOR);

    public Hook() {
	// choose the sensor
	hookLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidLoopIdx, timeOutMs);
	hookLift.setSensorPhase(sensorPhase);
	hookLift.setInverted(motorInvert);

	// set the peak, nominal outputs
	hookLift.configNominalOutputForward(0, timeOutMs);
	hookLift.configNominalOutputReverse(0, timeOutMs);
	hookLift.configPeakOutputForward(1, timeOutMs);
	hookLift.configPeakOutputReverse(-1, timeOutMs);

	// set allowable close-loop error
	hookLift.configAllowableClosedloopError(0, pidLoopIdx, timeOutMs);

	// set closed loop gains in slot0
	hookLift.config_kF(pidLoopIdx, kF, timeOutMs);
	hookLift.config_kP(pidLoopIdx, kP, timeOutMs);
	hookLift.config_kI(pidLoopIdx, kI, timeOutMs);
	hookLift.config_kD(pidLoopIdx, kD, timeOutMs);

	// set relative sensor to match absolute position
	int absolutePosition = hookLift.getSensorCollection().getPulseWidthPosition();
	/* mask out overflows, keep bottom 12 bits */
	absolutePosition &= 0xFFF;
	if (sensorPhase) {
	    absolutePosition *= -1;
	}
	if (motorInvert) {
	    absolutePosition *= -1;
	}
	/* set the quadrature (relative) sensor to match absolute */
	hookLift.setSelectedSensorPosition(absolutePosition, pidLoopIdx, timeOutMs);

	stop();
    }

    public double getDistance() {
	return Math.min(hookLift.getSensorCollection().getQuadraturePosition(),
		hookLift.getSensorCollection().getQuadraturePosition()) / COUNTS_PER_INCH;
    }

    @Override
    public void initDefaultCommand() {
	setDefaultCommand(new HookStop());
    }

    public void raise(double setPoint) {
	hookLift.set(setPoint);
    }

    public void stop() {
	hookLift.neutralOutput();
    }

    public void lower(double spd) {
	hookLift.set(spd);
    }

    public void resetEncoder() {
	hookLift.getSensorCollection().setQuadraturePosition(0, 100);
    }
}
