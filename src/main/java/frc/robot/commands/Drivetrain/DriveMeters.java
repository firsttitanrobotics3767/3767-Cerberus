package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants;

public class DriveMeters extends CommandBase {
    private final Drivetrain drivetrain;
    private double targetMeters;
    private TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final SimpleMotorFeedforward feedforward;

    public DriveMeters(double targetMeters, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.targetMeters = targetMeters;
        addRequirements(drivetrain);
        constraints = new TrapezoidProfile.Constraints(Constants.Drivetrain.Drive.kMaxDriveVel, Constants.Drivetrain.Drive.kMaxDriveAccel);
        controller = new ProfiledPIDController(
            Constants.Drivetrain.Drive.kP,
            Constants.Drivetrain.Drive.kI,
            Constants.Drivetrain.Drive.kD,
            constraints);

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Drive.kS, Constants.Drivetrain.Drive.kV);
        controller.setTolerance(0.5);
    }

    public DriveMeters(double targetMeters, Drivetrain drivetrain, double velocity, double acceleration) {
        this.drivetrain = drivetrain;
        this.targetMeters = targetMeters;
        addRequirements(drivetrain);
        constraints = new TrapezoidProfile.Constraints(velocity, acceleration);
        controller = new ProfiledPIDController(
            Constants.Drivetrain.Drive.kP,
            Constants.Drivetrain.Drive.kI,
            Constants.Drivetrain.Drive.kD,
            constraints);

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Drive.kS, Constants.Drivetrain.Drive.kV);
        controller.setTolerance(0.5);
    }

    @Override
    public void initialize() {
        // drivetrain.resetEncoders();
        controller.reset(drivetrain.getAverageMeters());
        controller.setGoal(drivetrain.getAverageMeters() + targetMeters);
    }

    @Override
    public void execute() {
        double targetSpeed = feedforward.calculate(controller.calculate(drivetrain.getAverageMeters()));
        drivetrain.arcadeDrive(targetSpeed, 0);
        SmartDashboard.putNumber("Drive Controller output", targetSpeed);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.arcadeDrive(0, 0);
        resetConstraints();
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }

    public void setConstraints(double velocity, double acceleration) {
        controller.setConstraints(new TrapezoidProfile.Constraints(velocity, acceleration));
    }

    public void resetConstraints() {
        controller.setConstraints(constraints);
    }

    public void setNewGoal(double targetMeters) {
        controller.setGoal(drivetrain.getAverageMeters() + targetMeters);
    }
}
