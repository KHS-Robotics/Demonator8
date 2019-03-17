package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Climber;

public class AutoClimb extends CommandGroup {
    public AutoClimb(Climber climber) {
        this.addSequential(new ClimbPID(climber));
        this.addSequential(new HoldFrontClimb(climber));
    }
}
