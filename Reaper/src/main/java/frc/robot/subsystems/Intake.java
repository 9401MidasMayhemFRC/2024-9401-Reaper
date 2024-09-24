package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private CANSparkMax m_motor = new CANSparkMax(9,MotorType.kBrushless);
    private SparkPIDController m_PID = m_motor.getPIDController();

    public Intake(){

        m_motor.restoreFactoryDefaults();
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(60);
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);
        
        m_PID.setP(0.0001);
        m_PID.setFF(1.0/5676.0);
        m_motor.burnFlash();

    }
    public Command runCmd(double percent) {
        return new InstantCommand(() -> run(percent));
    }

    public Command stopCmd() {
        return new InstantCommand(() -> stop());
    }

    public void run(double percent) {
        m_motor.set(percent);
    }

    public void stop() {
        m_motor.set(0.0);
    }


}
