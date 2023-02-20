package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class HomePivot extends CommandBase{
    private final Pivot pivot;

    public HomePivot(Pivot pivot) {
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.setPivotSpeed(-0.1);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.setPivotSpeed(0);
        pivot.resetPivotEncoder();
        pivot.setSoftLimits(10, 0);
        pivot.enableSoftLimits(true);
    }

    @Override
    public boolean isFinished() {
        return pivot.reverseLimitSwitch.get();
    }
}
