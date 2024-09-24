package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(16, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_PID = m_motor.getPIDController();
    // forward limit is 78 and is setpoint for scoring
    private double m_position = ElevatorConstants.kRestPose;

    public Elevator(){

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(40);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.0);//untested
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)78.0);//untested
        m_motor.setInverted(false);//untested
        m_motor.setIdleMode(IdleMode.kBrake);
        m_encoder.setPositionConversionFactor(1.0);//untested
        m_encoder.setVelocityConversionFactor(1.0);//untested
        m_PID.setP(0.1,0);//untested

        m_motor.burnFlash();
    }

    public void setPosition(double position){
        m_position = position;
    }

    public void zero(){
        m_encoder.setPosition(0.0);
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public double getPositionError(){
        return m_position-getPosition();
    }

    @Override
    public void periodic(){
        m_PID.setReference(m_position, ControlType.kPosition,0,0.0);
        SmartDashboard.putNumber("Elevator Setpoint", m_position);
        SmartDashboard.putNumber("Elevator Pose", getPosition());
        SmartDashboard.putNumber("Elevator Output", m_motor.getAppliedOutput());
    }


}