package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RackPinion extends SubsystemBase{
    
    private CANSparkMax m_motor = new CANSparkMax(10,MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkPIDController m_PID = m_motor.getPIDController();

    private double m_pose = 5.0;
    private boolean m_enablePID = true;

    private final double m_offest = 0.0;

    public RackPinion(){

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(20);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) 50.0);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) 2.0);
        m_motor.setInverted(false);
        
        m_PID.setP(0.1);
        m_motor.burnFlash();
        
    }

    public void setPose(double pose) {
        m_pose = pose - m_offest;
    }

    public void runRack(){
        m_enablePID = true;
    }

    public void stop(){
        m_enablePID = false;
        m_motor.stopMotor();
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public void zero(){
        m_enablePID = false;
        m_motor.set(-0.1);
    }

    public void setZero(){
        m_encoder.setPosition(-1.0);
    }

    public Command incrementHood(){
    return new InstantCommand(()-> setPose(m_pose + 5.0));
  }

  public Command decrementHood(){
    return new InstantCommand(()-> setPose(m_pose - 5.0));
  }

    @Override
    public void periodic(){
        if(m_enablePID){
            m_PID.setReference(m_pose, ControlType.kPosition);
        }
        SmartDashboard.putNumber("Rack Pose",getPosition());
        SmartDashboard.putNumber("Target Rack Pose", m_pose);
        SmartDashboard.putNumber("Rack Current", getCurrent());

    }
}
