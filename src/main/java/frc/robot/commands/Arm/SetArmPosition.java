package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants;

public class SetArmPosition extends CommandBase{
    private final Arm arm;
    private final double targetPosition;
    private final ProfiledPIDController controller;
    private TrapezoidProfile.Constraints constraints;
    private double initialDistanceToGoal;
    private boolean hasAccelerationBeenChanged = false;

    public SetArmPosition(double targetPosition, Arm arm) {
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(arm);
        constraints = new TrapezoidProfile.Constraints(Constants.Arm.kMaxVel, Constants.Arm.kMaxAccel);
        controller = new ProfiledPIDController(
            Constants.Arm.kP,
            Constants.Arm.kI,
            Constants.Arm.kD,
            constraints);
    }

    @Override
    public void initialize() {
        controller.reset(arm.getArmPosition());
        initialDistanceToGoal = targetPosition - arm.getArmPosition();
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
        double targetVolts = controller.calculate(arm.getArmPosition());
        arm.setArmVolts(targetVolts);
        SmartDashboard.putNumber("controller output", targetVolts);
    }

    @Override
    public void end(boolean isFinished) {
        arm.setArmVolts(0);
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
