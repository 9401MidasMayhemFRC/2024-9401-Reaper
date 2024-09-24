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
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    
    private CANSparkMax m_leftMotor = new CANSparkMax(14,MotorType.kBrushless);
    private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    private SparkPIDController m_leftPID = m_leftMotor.getPIDController();

    private CANSparkMax m_rightMotor = new CANSparkMax(15,MotorType.kBrushless);
    private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
    private SparkPIDController m_rightPID = m_rightMotor.getPIDController();

    private boolean m_PIDenabled = true;
    private double m_pose = 0.5;

    public Climber(){

        m_leftMotor.restoreFactoryDefaults();
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setInverted(true);
        m_leftMotor.enableVoltageCompensation(12);
        m_leftMotor.setSmartCurrentLimit(60);
        m_leftMotor.setSoftLimit(SoftLimitDirection.kForward,(float)ClimberConstants.kforwardSoftlimit);
        m_leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimberConstants.kreverseSoftlimit);

        m_leftPID.setP(0.0001);
        m_leftPID.setI(0);
        m_leftPID.setD(0);
        m_leftPID.setFF(1.0/5676.0);

        m_leftPID.setSmartMotionMaxAccel(10000.0,0);
        m_leftPID.setSmartMotionMaxVelocity(5000.0,0);

        m_leftMotor.burnFlash();

        m_rightMotor.restoreFactoryDefaults();
        m_rightMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setInverted(false);
        m_rightMotor.enableVoltageCompensation(12);
        m_rightMotor.setSmartCurrentLimit(60);
        m_rightMotor.setSoftLimit(SoftLimitDirection.kForward,(float)ClimberConstants.kforwardSoftlimit);
        m_rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimberConstants.kreverseSoftlimit);

        m_rightPID.setSmartMotionMaxAccel(10000.0,0);
        m_rightPID.setSmartMotionMaxVelocity(5000.0,0);

        m_rightPID.setP(0.0001);
        m_rightPID.setI(0);
        m_rightPID.setD(0);
        m_rightPID.setFF(1.0/5676);
        m_rightMotor.burnFlash();

    }

    public void extend(){
        m_pose = ClimberConstants.kforwardSoftlimit-5.0;
        m_PIDenabled = true;
    }

    public void retract(){
        m_pose = ClimberConstants.kreverseSoftlimit+5.0;
        m_PIDenabled = true;
    }

    public void setPose(double pose){
        m_pose = pose;
        m_PIDenabled = true;
    }

    public void zeroLeft(){
        m_PIDenabled = false;
        m_leftMotor.set(-.1);

    }
    public void zeroRight(){
        m_PIDenabled = false;
        m_rightMotor.set(-.1);

    }

    public void setLeftZero(){
        m_leftEncoder.setPosition(0.0);
    }

    public void setRightZero(){
        m_rightEncoder.setPosition(0.0);
    }

    public void stopLeft(){
        m_leftMotor.stopMotor();
    }

    public void stopRight(){
        m_rightMotor.stopMotor();
    }

    public double getLeftPose(){
        return m_leftEncoder.getPosition();
    }

    public double getRightPose(){
        return m_rightEncoder.getPosition();
    }

    public double getLeftCurrent(){
        return m_leftMotor.getOutputCurrent();
    }

    public double getRightCurrent(){
        return m_rightMotor.getOutputCurrent();
    }

    public void run(){
        m_PIDenabled = true;
    }

    public void stop(){
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
        m_PIDenabled = false;
    }

    @Override
    public void periodic() {
        
        if(m_PIDenabled){
            m_leftPID.setReference(m_pose,ControlType.kSmartMotion,0);
            m_rightPID.setReference(m_pose,ControlType.kSmartMotion,0);
        }

        SmartDashboard.putNumber("Climber Setpoint",m_pose);
        SmartDashboard.putNumber("Left Climber Current",getLeftCurrent());
        SmartDashboard.putNumber("Right Climber Current",getRightCurrent());
                SmartDashboard.putNumber("Left Climber Pose",getLeftPose());
        SmartDashboard.putNumber("Right Climber Pose",getRightPose());

    }

}
