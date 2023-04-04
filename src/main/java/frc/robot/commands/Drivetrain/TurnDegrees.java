package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;

public class TurnDegrees extends CommandBase {
    private final Drivetrain drivetrain;
    private double targetHeading;
    private TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public TurnDegrees(double targetHeading, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.targetHeading = targetHeading;
        addRequirements(drivetrain);
        constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.kMaxTurnVel, Constants.Drivetrain.kMaxTurnAccel);
        controller = new ProfiledPIDController(
            Constants.Drivetrain.kP,
            Constants.Drivetrain.kI,
            Constants.Drivetrain.kD,
            constraints);
        controller.enableContinuousInput(-180, 180);

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kS, Constants.Drivetrain.kV);
    }

    @Override
    public void initialize() {
        controller.reset(drivetrain.getGyroYaw());
        controller.setGoal(targetHeading);
    }

    @Override
    public void execute() {
        double targetSpeed = feedforward.calculate(controller.calculate(drivetrain.getGyroYaw()));
        drivetrain.arcadeDrive(0, -targetSpeed);
        SmartDashboard.putNumber("Controller output", -targetSpeed);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }
}
