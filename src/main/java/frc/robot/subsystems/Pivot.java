package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
// Vendor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

// WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.utils.Constants;
import frc.robot.utils.Dashboard;
// Utils
import frc.robot.utils.IDMap;

/** The rotation of the arm */
public class Pivot extends SubsystemBase{
    private final RobotContainer robotContainer;
    // Devices
    private final CANSparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder;
    public final DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public Boolean limitSwitchesEnabled = true, softLimitsEnabled = true, speedControl = false;
    public final Dashboard.Entry<Double> pivotSpeed, pivotPosition, pivotVoltage, pivotError, setpointDashboard;
    public double targetVolts = 0, volts = 0, forwardLimit = 20, reverseLimit = -83;

    public Pivot(RobotContainer robotContainer) {

        this.robotContainer = robotContainer;

        pivotSpeed = Dashboard.Entry.getDoubleEntry("Pivot Speed", 0);
        pivotPosition = Dashboard.Entry.getDoubleEntry("Pivot Position", 0);
        pivotVoltage = Dashboard.Entry.getDoubleEntry("Pivot Voltage", 0);
        pivotError = Dashboard.Entry.getDoubleEntry("Pivot Error", 0);
        setpointDashboard = Dashboard.Entry.getDoubleEntry("Setpoint", 0);

        // Pivot motor
        pivotMotor = new CANSparkMax(IDMap.CAN.pivot.ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);
        pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        // Pivot Encoder
        // pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        pivotEncoder.setPositionConversionFactor(Constants.Pivot.degreesPerRevolution);
        pivotEncoder.setInverted(true);

        // Pivot Limit Switches
        forwardLimitSwitch = new DigitalInput(IDMap.DIO.pivotForwardLimit.port);
        reverseLimitSwitch = new DigitalInput(IDMap.DIO.pivotReverseLimit.port);
    }

    @Override
    public void periodic() {
        double gravityCompensation = Constants.Pivot.kG * Math.cos(Units.degreesToRadians(pivotEncoder.getPosition()));
        volts = targetVolts;
        volts += gravityCompensation;
        if (limitSwitchesEnabled || softLimitsEnabled) {
            if (isAbleToMoveForward() && isAbleToMoveReverse()) {
                pivotMotor.setVoltage(volts);
            } else {
                if (isAbleToMoveForward() && volts > 0) {
                    pivotMotor.setVoltage(volts);
                } else if (isAbleToMoveReverse() && volts < 0) {
                    pivotMotor.setVoltage(volts);
                } else {pivotMotor.setVoltage(0);}
            }
                
        } else {
            pivotMotor.setVoltage(volts);
        }
        pivotVoltage.put(volts);
        pivotPosition.put(getPivotPosition());
        pivotError.put(error);
        SmartDashboard.putNumber("Pivot current", pivotMotor.getOutputCurrent());
    }

    // Motor methods
    public void setPivotVolts(double volts) {
        if (volts > 0.05 || volts < -0.05) {
            speedControl = true;
            targetVolts = volts;
        } else {
            speedControl = false;
            targetVolts = 0;
        }

    }

    public void setPivotDashboard() {
        pivotMotor.set(Dashboard.pivotSpeed.get());
    }

    // Encoder methods
    public double getPivotPosition() {
        return pivotEncoder.getPosition() > 180 ? pivotEncoder.getPosition() - 360 : pivotEncoder.getPosition();
    }

    public double getPivotRate() {
        return pivotEncoder.getVelocity();
    }

    public void resetPivotEncoder(double position) {
        // pivotEncoder.setZeroOffset(position);
    }

    // Limits
    public void enableSoftLimits(boolean enabled) {
        softLimitsEnabled = enabled;
    }

    public void setSoftLimits(double forwardLimit, double reverseLimit) {
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
    }

    public void enableLimitSwitches(boolean enabled) {
        limitSwitchesEnabled = enabled;
    }

    public boolean getForwardLimitSwitchPressed() {
        return forwardLimitSwitch.get();
    }

    public boolean getReverseLimitSwitchPressed() {
        return reverseLimitSwitch.get();
    }

    public boolean getForwardLimitExceeded() {
        return getPivotPosition() > forwardLimit;
    }

    public boolean getReverseLimitExceeded() {
        return getPivotPosition < reverseLimit;
    }

    public boolean isAbleToMoveForward() {
        return !(
            getForwardLimitExceeded() ||
            getForwardLimitSwitchPressed
        );
    }

    public boolean isAbleToMoveReverse() {
        return !(
            getReverseLimitExceeded() ||
            getReverseLimitSwitchPressed()
        );
    }
}
