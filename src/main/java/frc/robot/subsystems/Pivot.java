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
    // private final RelativeEncoder pivotEncoder;
    private final AbsoluteEncoder pivotEncoder;
    public final DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    public Boolean limitSwitchesEnabled = true, isCalibrated = false, speedControl = false;
    public final Dashboard.Entry<Double> pivotSpeed, pivotPosition, pivotVoltage, pivotError, setpointDashboard;
    public double setpoint = 0, targetVolts = 0, volts = 0;

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
        setpoint = setpointDashboard.get();
        double error = setpoint + pivotEncoder.getPosition();
        double gravityCompensation = Constants.Pivot.kG * Math.cos(Units.degreesToRadians(pivotEncoder.getPosition()));
        // volts = (Constants.Pivot.kP * -error);
        // if (error < Constants.Pivot.dampenerLimit) {
        //     volts = (error * Constants.Pivot.dampeningFactor) + Constants.Pivot.travelVolts;
        // } else {
        //     volts = Constants.Pivot.travelVolts;
        // }
        // if (speedControl) {
        //     volts = targetVolts;
        // }
        volts = targetVolts;
        volts += gravityCompensation;
        if (limitSwitchesEnabled) {
            if (forwardLimitSwitch.get() || reverseLimitSwitch.get()) {
                if (forwardLimitSwitch.get() && volts > 0) {
                    pivotMotor.setVoltage(volts);
                } else if (reverseLimitSwitch.get() && volts < 0) {
                    pivotMotor.setVoltage(volts);
                } else {pivotMotor.setVoltage(0);}
            } else {
                pivotMotor.setVoltage(volts);
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
        // return pivotEncoder.getPosition();
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
