package frc.robot.commands.Pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants;

public class SetPivotPosition extends CommandBase{
    private final Pivot pivot;
    private final double targetPosition;
    private final ProfiledPIDController controller;
    private TrapezoidProfile.Constraints constraints;
    private double initialDistanceToGoal;
    // private boolean hasAccelerationBeenChanged = false;

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
        // When the pivot passes the halfway point between start and goal, cut
        // the acceleration in half.
        // if (!hasAccelerationBeenChanged && isPivotPastHalfwayPoint()) {
        //     reduceControllerAccelerationByFactorOf(2);
        //     hasAccelerationBeenChanged = true;
        // }
        double targetVolts = controller.calculate(pivot.getPivotPosition());
        pivot.setPivotVolts(targetVolts);
        SmartDashboard.putNumber("controller output", targetVolts);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.setPivotVolts(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Reduce the controller's acceleration constraint by the given factor.
     */
    private void reduceControllerAccelerationByFactorOf(double factor) {
        constraints = new TrapezoidProfile.Constraints(Constants.Pivot.kMaxVel, Constants.Pivot.kMaxAccel / factor);
        controller.setConstraints(constraints);
    }

    /**
     * Check to see if the pivot is past the halfway point, according to our
     * controller.
     *
     * @return True if the pivot is past the halfway point, false otherwise.
     */
    private boolean isPivotPastHalfwayPoint() {
        return controller.getGoal().position - controller.getSetpoint().position < (1/2) * initialDistanceToGoal;
    }
    
}
