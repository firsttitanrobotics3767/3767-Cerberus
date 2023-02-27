package frc.robot.subsystems;

// Vendor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPILib
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Constants;
import frc.robot.utils.Dashboard;
// Utils
import frc.robot.utils.IDMap;

/** The rotation of the arm */
public class Pivot extends SubsystemBase{
    // Devices
    private final CANSparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    public final DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public Boolean limitSwitchesEnabled = true, isCalibrated = false, speedControl = false;
    public final Dashboard.Entry<Double> pivotSpeed, pivotPosition, pivotVoltage, pivotError;
    public double setpoint = 0, targetVolts = 0, volts = 0;

    public Pivot() {

        pivotSpeed = Dashboard.Entry.getDoubleEntry("Pivot Speed", 0);
        pivotPosition = Dashboard.Entry.getDoubleEntry("Pivot Position", 0);
        pivotVoltage = Dashboard.Entry.getDoubleEntry("Pivot Voltage", 0);
        pivotError = Dashboard.Entry.getDoubleEntry("Pivot Error", 0);

        // Pivot motor
        pivotMotor = new CANSparkMax(IDMap.CAN.pivot.ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);

        // Pivot Encoder
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPositionConversionFactor(Constants.Pivot.degreesPerTick);

        // Pivot Limit Switches
        forwardLimitSwitch = new DigitalInput(IDMap.DIO.pivotForwardLimit.port);
        reverseLimitSwitch = new DigitalInput(IDMap.DIO.pivotReverseLimit.port);


    }

    @Override
    public void periodic() {
        double error = setpoint - pivotEncoder.getPosition();
        double gravityCompensation = Constants.Pivot.kG * Math.cos(pivotEncoder.getPosition());
        volts = (Constants.Pivot.kP * error);
        if (speedControl) {
            volts = targetVolts;
        }
        volts += gravityCompensation;
        if (limitSwitchesEnabled) {
            if (forwardLimitSwitch.get() || reverseLimitSwitch.get()) {
                if (forwardLimitSwitch.get() && volts < 0) {
                    pivotMotor.setVoltage(volts);
                } else if (reverseLimitSwitch.get() && volts > 0) {
                    pivotMotor.setVoltage(volts);
                } else {pivotMotor.setVoltage(0);}
            } else {
                pivotMotor.setVoltage(volts);
            }
        } else {
            pivotMotor.setVoltage(volts);
        }
        pivotVoltage.put(volts);
        pivotPosition.put(pivotEncoder.getPosition());
        pivotError.put(error);
    }

    // TODO: position control

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

    public void setPivotPosition(double setpoint) {
        this.setpoint = setpoint;
    }

    public void addPivotPosition(double speed) {
        setpoint += speed;
    }

    public void setPivotDashboard() {
        pivotMotor.set(Dashboard.pivotSpeed.get());
    }

    // Encoder methods
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public double getPivotRate() {
        return pivotEncoder.getVelocity();
    }

    public void resetPivotEncoder() {
        pivotEncoder.setPosition(0);
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
        limitSwitchesEnabled = enabled;
    }    
}
