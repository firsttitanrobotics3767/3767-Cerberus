package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class HomePivot extends CommandBase{
    private final Pivot pivot;

    public HomePivot(Pivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
        setName("Home Pivot");
    }

    @Override
    public void initialize() {
        pivot.enableSoftLimits(false);
        pivot.setPivotVolts(-1);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.setPivotVolts(0);
        pivot.resetPivotEncoder();
        // TODO: set limits and position to use degrees and level arm
        pivot.setSoftLimits(55, 0.05);
        pivot.enableSoftLimits(true);
    }

    @Override
    public boolean isFinished() {
        return pivot.forwardLimitSwitch.get();
    }
}
