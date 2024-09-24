package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LEDs;

public class Feeder extends SubsystemBase{

    private DigitalInput m_prox = new DigitalInput(0);
    
    private CANSparkMax m_feedMotor = new CANSparkMax(13,MotorType.kBrushless);
    private RelativeEncoder m_feedEncoder = m_feedMotor.getEncoder();
    private SparkPIDController m_feedPID = m_feedMotor.getPIDController();
    private LEDs m_leds;

    private double m_feedVelo = 0.0;

    public Feeder(LEDs m_led){

        m_leds = m_led;

        m_feedMotor.restoreFactoryDefaults();
        m_feedMotor.enableVoltageCompensation(12.0);
        m_feedMotor.setSmartCurrentLimit(30);
        m_feedMotor.setInverted(true);
        m_feedMotor.setIdleMode(IdleMode.kBrake);

        m_feedPID.setP(0.0001);
        m_feedPID.setFF(1.0/5676.0);

                m_feedMotor.burnFlash();

    }

    public boolean getProx(){
        return m_feedMotor.getOutputCurrent() >= 25.0;
    }

    public void setFeedVelo(double power){
        m_feedMotor.set(power);
    }

    public void stopFeed(){
        m_feedMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Feed Velo.",m_feedVelo);
        SmartDashboard.putNumber("Actual Feed Velo.",m_feedEncoder.getVelocity());
        SmartDashboard.putBoolean("Prox.", getProx());

        // if(getProx()){
        //     m_leds.setGreen();
        // } else {
        //     m_leds.setTeamColors();
        // }
    }

}
