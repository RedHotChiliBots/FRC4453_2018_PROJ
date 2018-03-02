package org.usfirst.frc.team4453.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CrossAutoLine extends CommandGroup {
    
    public CrossAutoLine() {
        addSequential(new RobotDriveDistance(FieldConstants.AUTO_LINE_DIST)); // 5 inches
    }
}
