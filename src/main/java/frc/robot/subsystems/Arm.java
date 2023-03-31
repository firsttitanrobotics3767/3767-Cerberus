package frc.robot.subsystems;

// Vendor libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPILib
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Utils
import frc.robot.utils.Constants;
import frc.robot.utils.Dashboard;
import frc.robot.utils.Dashboard.Entry;
import frc.robot.utils.IDMap;

/** The extension of the arm */
public class Arm extends SubsystemBase {
    /** Extension motor. Positive values will extend. */
    private final CANSparkMax armMotor;
    private final Encoder armEncoder;
    public final DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public Boolean limitSwitchesEnabled = true, softLimitsEnabled = true;
    public final Entry<Double> armVoltage, armPosition;
    public double targetVolts = 0, volts = 0, forwardLimit = 84, reverseLimit = 0.5;
    public Pivot pivot = null;

    public Arm() {

        armVoltage = Entry.getDoubleEntry("Arm Target Speed", 0);
        armPosition = Entry.getDoubleEntry("Arm Position", 0);

        // Arm motor
        armMotor = new CANSparkMax(IDMap.CAN.arm.ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(true);

        // Arm encoder
        armEncoder = new Encoder(IDMap.DIO.armEncoderA.port, IDMap.DIO.armEncoderB.port);
        armEncoder.setDistancePerPulse(Constants.Arm.distancePerPulse);

        // Arm limit switches
        forwardLimitSwitch = new DigitalInput(IDMap.DIO.armForawrdLimit.port);
        reverseLimitSwitch = new DigitalInput(IDMap.DIO.armReverseLimit.port);
    }

    @Override
    public void periodic() {
        double gravityCompensation = Constants.Arm.kG * Math.sin(Units.degreesToRadians(pivot.getPivotPosition()));
        volts = targetVolts + gravityCompensation;
        if (isAbleToMoveForward() && isAbleToMoveReverse()) {
            armMotor.setVoltage(volts);
        } else {
            if (isAbleToMoveForward() && volts > 0) {
                armMotor.setVoltage(volts);
            } else if (isAbleToMoveReverse() && volts < 0) {
                armMotor.setVoltage(volts);
            } else {armMotor.setVoltage(0);}
        }
        SmartDashboard.putBoolean("forward", isAbleToMoveForward());
        SmartDashboard.putBoolean("back", isAbleToMoveReverse());
        armVoltage.put(volts);
        armPosition.put(armEncoder.getDistance());
    }

    // Motor methods
    public void setArmVolts(double volts) {
        targetVolts = (volts > 0.1 || volts < -0.1)? volts : 0;
    }

    public void setArmDashboard() {
        armMotor.set(Dashboard.armSpeed.get());
    }

    // Encoder methods
    public double getArmPosition() {
        return armEncoder.getDistance();
    }

    public double getArmVelocity() {
        return armEncoder.getRate();
    }

    public void resetArmEncoder() {
        armEncoder.reset();
    }

    // Limtis

    /**
     * Enables both encoder limits, providing a soft stop on the extension of the arm
     * @param enable set true to enable soft limits (enabled by default)
     */
    public void enableSoftlimits(boolean enabled) {
        softLimitsEnabled = enabled;
    }

    /**
     * Sets the soft limit values.
     * @param forwardLimit The value of the encoder that will stop the motor from extending.
     * @param reverseLimit The value of the encoder that will stop the motor from retracting.
     */
    public void setSoftLimits(double forwardLimit, double reverseLimit) {
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
    }

    /**
     * Enables both limit switches, providing an electrical stop on the extension and retraction of the arm.
     * @param enabled Set true to enable limit switch stops (enabled by default).
     */
    public void enableLimitSwitches(boolean enabled) {
        limitSwitchesEnabled = enabled;
    }

    public boolean getForwardLimitSwitchPressed() {
        return forwardLimitSwitch.get();
    }

    public boolean getReverseLimitSwitchPressed() {
        return !reverseLimitSwitch.get();
    }

    public boolean getForwardLimitExceeded() {
        return getArmPosition() >= forwardLimit;
    }

    public boolean getReverseLimitExceeded() {
        return getArmPosition() <= reverseLimit;
    }

    /**
     * Checks all forward limits, will return false if it is not able to move.
     * Accounts for whether or not each limit is enabled.
     * @return True if there are no limits exceeded.
     */
    public boolean isAbleToMoveForward() {
        return !(
            // If soft limit is enabled, check it. Otherwise assume limit is not exceeded.
            softLimitsEnabled? getForwardLimitExceeded() : false ||
            // If limit switch is enabled, check it. Otherwise, assume it is not pressed.
            limitSwitchesEnabled? getForwardLimitSwitchPressed() : false
        );
    }

    /**
     * Checks all reverse limits, will return false if it is not able to move.
     * Accounts for whether or not each limit is enabled.
     * @return True if there are no limits exceeded.
     */
    public boolean isAbleToMoveReverse() {
        return !(
            // If soft limit is enabled, check it. Otherwise, assume limit is not exceeded.
            softLimitsEnabled? getReverseLimitExceeded() : false ||
            // If limit switch is enabled, check it. Otherwise, assume it is not pressed.
            limitSwitchesEnabled? getReverseLimitSwitchPressed() : false
        );
    }

}
