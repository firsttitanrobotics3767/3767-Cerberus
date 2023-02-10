package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IDMap.CAN;

/**
 * Drivetrain motors, as well as camera and odometry
 */
public class Drivetrain extends SubsystemBase{
    private final CANSparkMax leftFront, leftBack, rightFront, rightBack;

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
    }
}