package frc.robot.subsystems;

import java.io.IOException;

// Vendor Libraries
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPILib
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Utils
import frc.robot.utils.Dashboard;
import frc.robot.utils.IDMap.CAN;
import frc.robot.utils.Constants;

/**
 * Drivetrain motors, as well as camera and odometry
 */
public class Drivetrain extends SubsystemBase{
    // Devices
    private final CANSparkMax leftFront, leftBack, rightFront, rightBack;
    private final RelativeEncoder leftDriveEncoder, rightDriveEncoder;
    private final AHRS gyro;
    private final PhotonCamera camera;

    // Utils
    public final DifferentialDrive differentialDrive;
    private final SlewRateLimiter throttleLimiter;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;


    public Drivetrain() {

        // Drivetrain Motor Configuration
        // Left Front 
        leftFront = new CANSparkMax(CAN.leftFront.ID, MotorType.kBrushless);
        leftFront.restoreFactoryDefaults();
        leftFront.setIdleMode(IdleMode.kBrake);
        leftFront.setInverted(true);

        // Left Back
        leftBack = new CANSparkMax(CAN.leftBack.ID, MotorType.kBrushless);
        leftBack.restoreFactoryDefaults();
        leftBack.setIdleMode(IdleMode.kBrake);
        leftBack.follow(leftFront);

        // Right Front 
        rightFront = new CANSparkMax(CAN.rightFront.ID, MotorType.kBrushless);
        rightFront.restoreFactoryDefaults();
        rightFront.setIdleMode(IdleMode.kBrake);
        rightFront.setInverted(false);

        // Right Back
        rightBack = new CANSparkMax(CAN.rightBack.ID, MotorType.kBrushless);
        rightBack.restoreFactoryDefaults();
        rightBack.setIdleMode(IdleMode.kBrake);
        rightBack.follow(rightFront);


        // Drive Encoders
        leftDriveEncoder = leftFront.getEncoder();
        rightDriveEncoder = rightFront.getEncoder();
        resetEncoders();

        // Gyro
        gyro = new AHRS(SerialPort.Port.kUSB1);
        // gyro.reset();

        // Camera
        camera = new PhotonCamera(Constants.cameraName);

        // Odometry
        differentialDrive = new DifferentialDrive(leftFront, rightFront);
        throttleLimiter = new SlewRateLimiter(Constants.Drivetrain.throttleLimiter);
        odometry = new DifferentialDriveOdometry(getGyroRotation2d(), getLeftDistanceRaw(), getRightDistanceRaw());
        poseEstimator = new DifferentialDrivePoseEstimator(
            Constants.Drivetrain.driveKinematics,
            getGyroRotation2d(),
            0,
            0,
            new Pose2d());
        
        // Field
        field = new Field2d();
        Dashboard.putSendable("Field", field);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
        SmartDashboard.putBoolean("on charging station", getGyroPitch() > 13);
        SmartDashboard.putNumber("Gyro Yaw", getGyroAngle());
        SmartDashboard.putNumber("Average Meters", getAverageMeters());
        odometry.update(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
        field.setRobotPose(getPose2d());
        SmartDashboard.putNumber("Chassis Speed", getLeftVelocityMetersPerSecond());
        SmartDashboard.putNumber("Distance", getLeftDistanceMeters());
    }

    // Drive methods
    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        differentialDrive.arcadeDrive(forwardSpeed, turnSpeed);
    }

    public void arcadeDriveThrottleLimited(double forwardSpeed, double turnSpeed) {
        differentialDrive.arcadeDrive(throttleLimiter.calculate(forwardSpeed), turnSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFront.setVoltage(leftVolts);
        rightFront.setVoltage(rightVolts);
        differentialDrive.feed();
        SmartDashboard.putNumber("volts", rightVolts);
    }


    // Encoder Methods
    public double getLeftDistanceRaw() {
        // return leftDriveEncoder.getPosition() * Constants.Drivetrain.metersPerRev;
        return leftDriveEncoder.getPosition();
    }

    public double getRightDistanceRaw() {
        // return rightDriveEncoder.getPosition() * Constants.Drivetrain.metersPerRev;
        return rightDriveEncoder.getPosition();
    }

    public double getLeftDistanceMeters() {
        return leftDriveEncoder.getPosition() * Constants.Drivetrain.metersPerRev;
    }

    public double getRightDistanceMeters() {
        return rightDriveEncoder.getPosition() * Constants.Drivetrain.metersPerRev;
    }

    public double getAverageRawPosition() {
        return (getLeftDistanceRaw() + getRightDistanceRaw()) / 2;
    }

    public double getAverageMeters() {
        return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2;
    }

    public double getLeftVelocityMetersPerSecond() {
        return (leftDriveEncoder.getVelocity() * Constants.Drivetrain.metersPerRev) / 60;
    }

    public double getRightVelocityMetersPerSecond() {
        return (rightDriveEncoder.getVelocity() * Constants.Drivetrain.metersPerRev) / 60;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSecond(),
            getRightVelocityMetersPerSecond()
        );
    }

    public void setLeftEncoderPosition(double position) {
        leftDriveEncoder.setPosition(position);
    }

    public void setRightEncoderPosition(double position) {
        rightDriveEncoder.setPosition(position);
    }

    public void resetEncoders() {
        leftDriveEncoder.setPosition(0);
        rightDriveEncoder.setPosition(0);
    }


    // Gyro Methods
    public double getGyroAngle() {
        // return gyro.getYaw();
        return gyro.getAngle() * Constants.Drivetrain.gyroAdjustment;
    }

    public double getGyroPitch() {
        return gyro.getPitch();
    }

    public Rotation2d getGyroRotation2d() {
        return gyro.getRotation2d().times(Constants.Drivetrain.gyroAdjustment);
    }

    public void resetGyro() {
        gyro.reset();
    }


    // Odometry Methods
    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public Command getRamseteCommand(String trajName, boolean resetPose) {
        Trajectory traj = PathPlanner.loadPath(trajName, Constants.Drivetrain.pathConstraints);
        SequentialCommandGroup commands = new SequentialCommandGroup();
        if (resetPose) commands.addCommands(new InstantCommand(() -> setOdometry(traj.getInitialPose())));
        commands.addCommands(new InstantCommand(() -> field.getObject("traj").setTrajectory(traj)));
        

        commands.addCommands(new RamseteCommand(
            traj, 
            this::getPose2d,
            new RamseteController(2, 0.7),
            new SimpleMotorFeedforward(
                Constants.Drivetrain.Trajectory.kS,
                Constants.Drivetrain.Trajectory.kV),
            Constants.Drivetrain.driveKinematics,
            this::getWheelSpeeds,
            new PIDController(Constants.Drivetrain.Trajectory.kLeftP, 0.0, 0.0),
            new PIDController(Constants.Drivetrain.Trajectory.kLeftP, 0.0, 0.0),
            this::tankDriveVolts,
            this));
        return commands;
    }

    public void updatePoseEstimate() {
        poseEstimator.update(getGyroRotation2d(), getLeftDistanceRaw(), getRightDistanceRaw());
        var result = getCameraResult();
        if (result.hasTargets()) {
            double imageCaptureTime = result.getTimestampSeconds();
            var camToTarget = result.getBestTarget().getBestCameraToTarget();
            var camPose = fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().transformBy(camToTarget.inverse());
            poseEstimator.addVisionMeasurement(camPose.toPose2d().transformBy(Constants.cameraTransform), imageCaptureTime);
        }

        setOdometry(poseEstimator.getEstimatedPosition());
    }

    public void setOdometry(Pose2d newPose) {
        odometry.resetPosition(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters(), newPose);
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }

    public void resetAll() {
        resetEncoders();
        resetGyro();
        resetOdometry();
    }


    // Camera Methods
    public PhotonPipelineResult getCameraResult() {
        return camera.getLatestResult();
    }

}