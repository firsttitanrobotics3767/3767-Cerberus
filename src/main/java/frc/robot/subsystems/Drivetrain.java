package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Dashboard;
import frc.robot.utils.IDMap.CAN;

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
    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;


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
        gyro = new AHRS();
        gyro.reset();

        // Camera
        camera = new PhotonCamera("OV5647");

        // Odometry
        differentialDrive = new DifferentialDrive(leftFront, rightFront);
        odometry = new DifferentialDriveOdometry(getGyroRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
        poseEstimator = new DifferentialDrivePoseEstimator(
            Constants.Drivetrain.driveKinematics,
            getGyroRotation2d(),
            0,
            0,
            null);
        
        // Field
        Dashboard.putSendable("Field", field);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
        


    }


}