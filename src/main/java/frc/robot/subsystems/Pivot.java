package frc.robot.subsystems;

// Vendor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

// Utils
import frc.robot.utils.IDMap;

public class Pivot extends SubsystemBase{
    // Devices
    /** Drive motor for the pivot. Positive values will raise the arm. */
    private final CANSparkMax pivotMotor;
    /** Pivot encoder. Mounted directly to the axle, plugged into DIO on Rio. */
    private final Encoder pivotEncoder;
    /** Forward and reverse limit switches for the pivot. Connected to motor controller */
    public final SparkMaxLimitSwitch forwardLimitSwitch, reverseLimitSwitch;

    public Pivot() {

        // Pivot motor
        pivotMotor = new CANSparkMax(IDMap.CAN.pivot.ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);

        // Pivot Encoder
        pivotEncoder = new Encoder(IDMap.DIO.pivotEncoderA.port, IDMap.DIO.pivotEncoderB.port);

        // Pivot Limit Switches
        forwardLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    @Override
    public void periodic() {

    }

    //TODO: position control

    // Motor methods
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    // Encoder methods
    public double getPivotPosition() {
        return pivotEncoder.getDistance();
    }

    public double getPivotRate() {
        return pivotEncoder.getRate();
    }

    public void resetPivot() {
        pivotEncoder.reset();
    }

    // Limits
    public void enableSoftLimits(boolean enabled) {
        pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    public void setSoftLimits(double forwardLimit, double reverseLimit) {
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float)forwardLimit);
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)reverseLimit);
    }

    public void enableLimitSwitches(boolean enabled) {
        forwardLimitSwitch.enableLimitSwitch(enabled);
        reverseLimitSwitch.enableLimitSwitch(enabled);
    }    
}
