package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

public class SetPivotPosition extends CommandBase{
    private final Pivot pivot;
    private double targetPosition;
    private double initialDistanceToGoal;
    private boolean hasAccelerationBeenChanged = false;
    private final ProfiledPIDController controller;
    private final TrapezoidProfile.Constraints constraints;

    public SetPivotPosition(double targetPosition, Pivot pivot) {
        this.pivot = pivot;
        this.targetPosition = targetPosition;
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
        initialDistanceToGoal = targetPosition - pivot.getPivotPosition();
        controller.setGoal(targetPosition);
    }

    @Override
    public void execute() {
        updateControllerConstraints();
        double targetVolts = controller.calculate(pivot.getPivotPosition());
        pivot.setPivotVolts(targetVolts);
        SmartDashboard.putNumber("controller output", targetVolts);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Check to see if the pivot has passed the halfway point, according to our
     * controller. If we have, update the controller constraints and reduce the
     * maximum acceleration. This is to avoid abrupt stops (which are more
     * taxing on the robot compared to abrupt starts).
     */
    private void updateControllerConstraints() {
        if (!hasAccelerationBeenChanged && isControllerPastHalfwayPoint()) {
            controller.setConstraints(new TrapezoidProfile.Constraints(Constants.Pivot.kMaxVel, Constants.Pivot.kMaxAccel / 2));
            hasAccelerationBeenChanged = true;
        }
    }

    /**
     * Check to see if the pivot is past the halfway point, according to our
     * controller.
     * 
     * @return True if the pivot is past the halfway point, false otherwise.
     */
    private boolean isControllerPastHalfwayPoint() {
        return controller.getGoal().position - controller.getSetpoint().position < (1/2) * initialDistanceToGoal;
    }
}
