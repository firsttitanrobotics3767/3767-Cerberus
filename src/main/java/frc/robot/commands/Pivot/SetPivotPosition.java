package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

public class SetPivotPosition extends CommandBase{
    private final Pivot pivot;
    private double position;
    private final ProfiledPIDController controller;
    private final TrapezoidProfile.Constraints constraints;

    public SetPivotPosition(double position, Pivot pivot) {
        this.pivot = pivot;
        this.position = position;
        addRequirements(pivot);
        constraints = new TrapezoidProfile.Constraints(Constants.Pivot.kMaxVel, Constants.Pivot.kMaxAccel);
        controller = new ProfiledPIDController(
            Constants.Pivot.kP,
            Constants.Pivot.kI,
            Constants.Pivot.kD,
            constraints);
    }

    @Override
    public void initialize() {
        controller.reset(pivot.getPivotPosition());
        controller.setGoal(position);
    }

    @Override
    public void execute() {
        pivot.setPivotVolts(controller.calculate(pivot.getPivotPosition()));
        SmartDashboard.putNumber("controller output", controller.calculate(pivot.getPivotPosition()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
