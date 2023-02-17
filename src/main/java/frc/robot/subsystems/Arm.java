package frc.robot.subsystems;

// Vendor libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPILib
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Utils
import frc.robot.utils.Dashboard;
import frc.robot.utils.IDMap;

/** The extension of the arm */
public class Arm extends SubsystemBase {
    /** Extension motor. Positive values will extend. */
    private final CANSparkMax armMotor;
    private final Encoder armEncoder;
    public final SparkMaxLimitSwitch forwardLimitSwitch, reverseLimitSwitch;

    public Arm() {
        // Arm motor
        armMotor = new CANSparkMax(IDMap.CAN.arm.ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(true);

        // Arm encoder
        armEncoder = new Encoder(IDMap.DIO.armEncoderA.port, IDMap.DIO.armEncoderB.port);

        // Arm limit switches
        forwardLimitSwitch = armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    @Override
    public void periodic() {

    }

    // TODO: position control

    // Motor methods
    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    public void setArmVolts(double volts) {
        armMotor.setVoltage(volts);
    }

    public void setArmDashboard() {
        armMotor.set(Dashboard.armSpeed.get());
    }

    // Encoder methods
    public double getArmPosition() {
        return armEncoder.getDistance();
    }

    public double getArmRate() {
        return armEncoder.getRate();
    }

    public void resetArmEncoder() {
        armEncoder.reset();
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
        forwardLimitSwitch.enableLimitSwitch(enabled);
        reverseLimitSwitch.enableLimitSwitch(enabled);
    }

}
