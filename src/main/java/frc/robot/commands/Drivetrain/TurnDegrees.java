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
    private double targetDegrees;
    private TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public TurnDegrees(double targetDegrees, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.targetDegrees = targetDegrees;
        addRequirements(drivetrain);
        constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.Turn.kMaxTurnVel, Constants.Drivetrain.Turn.kMaxTurnAccel);
        controller = new ProfiledPIDController(
            Constants.Drivetrain.Turn.kP,
            Constants.Drivetrain.Turn.kI,
            Constants.Drivetrain.Turn.kD,
            constraints);
        controller.enableContinuousInput(-180, 180);

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Turn.kS, Constants.Drivetrain.Turn.kV);
    }

    @Override
    public void initialize() {
        controller.reset(drivetrain.getGyroAngle());
        controller.setGoal(drivetrain.getGyroAngle() - targetDegrees);
    }

    @Override
    public void execute() {
        double targetSpeed = feedforward.calculate(controller.calculate(drivetrain.getGyroAngle()));
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
