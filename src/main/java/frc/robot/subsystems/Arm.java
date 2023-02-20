package frc.robot.subsystems;

// Vendor libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
// WPILib
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Utils
import frc.robot.utils.Dashboard;
import frc.robot.utils.IDMap;

/** The extension of the arm */
public class Arm extends SubsystemBase {
    /** Extension motor. Positive values will extend. */
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    public final DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public Boolean limitSwitchesEnabled = true;

    public Arm() {
        // Arm motor
        armMotor = new CANSparkMax(IDMap.CAN.arm.ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(true);

        // Arm encoder
        armEncoder = armMotor.getEncoder();

        // Arm limit switches
        forwardLimitSwitch = new DigitalInput(IDMap.DIO.armForawrdLimit.port);
        reverseLimitSwitch = new DigitalInput(IDMap.DIO.armReverseLimit.port);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("port " + forwardLimitSwitch.getChannel(), forwardLimitSwitch.get());
        SmartDashboard.putBoolean("port " + reverseLimitSwitch.getChannel(), reverseLimitSwitch.get());
    }

    // TODO: position control

    // Motor methods
    public void setArmSpeed(double speed) {
        if (limitSwitchesEnabled) {
            if (forwardLimitSwitch.get() || reverseLimitSwitch.get()) {
                if (forwardLimitSwitch.get() && speed < 0) {
                    armMotor.set(speed);
                } else if (reverseLimitSwitch.get() && speed > 0) {
                    armMotor.set(speed);
                } else {armMotor.set(0);}
            } else {
                armMotor.set(speed);
            }
        } else {
            armMotor.set(speed);
        }
        SmartDashboard.putNumber("Arm Traget Speed", speed);
    }

    public void setArmVolts(double volts) {
        armMotor.setVoltage(volts);
    }

    public void setArmDashboard() {
        armMotor.set(Dashboard.armSpeed.get());
    }

    // Encoder methods
    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public double getArmRate() {
        return armEncoder.getVelocity();
    }

    public void resetArmEncoder() {
        armEncoder.setPosition(0);
    }

    // Limtis
    /**
     * Enables both encoder limits, providing a soft stop on the extension of the arm
     * @param enable set true to enable soft limits
     */
    public void enableSoftlimits(boolean enable) {
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    /**
     * Sets the soft limit values
     * @param forwardLimit value of encoder that will stop the motor from extending
     * @param reverseLimit value of encoder that will stop the motor from retracting
     */
    public void setSoftLimits(double forwardLimit, double reverseLimit) {
        armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)forwardLimit);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)reverseLimit);
    }

    /**
     * Enables both limit switches, providing an electrical stop on the extension of the arm
     * @param enabled set true to enable limti switch stops (enabled by default)
     */
    public void enableLimitSwitches(boolean enabled) {
        limitSwitchesEnabled = enabled;
    }

}
