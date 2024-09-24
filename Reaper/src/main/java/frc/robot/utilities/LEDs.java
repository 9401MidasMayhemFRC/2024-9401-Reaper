package frc.robot.utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean m_ledOn = true;
    private BooleanSupplier m_auto;

    public LEDs(int port, int length, BooleanSupplier auto){

        m_auto = auto;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        setTeamColors();

    }

    public void setPurple(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setLED(i,Color.kDarkViolet);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setWhite(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,255,255,255);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setTeal(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setLED(i, Color.kDarkTurquoise);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setBlack(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setHSV(i,0,0,0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setRedWhite(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            if ((i % 2) == 0){
                m_ledBuffer.setRGB(i,255,0,0);
            } else{
                m_ledBuffer.setRGB(i,255,255,255);
            }
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void setGreen(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,0,255,0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setGold(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,255,65,0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setYellow(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,255,255,0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setRed(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,255,0,0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setCherries(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            if ((i % 2) == 0){
                m_ledBuffer.setRGB(i,255,0,0);
            } else{
                m_ledBuffer.setRGB(i,0,0,255);
            }
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void setTeamColors(){
        for(int i = 0; (i + 5) < m_ledBuffer.getLength(); i = i + 5){
            if ((i % 10) == 0){
                m_ledBuffer.setLED(i,Color.kDarkViolet);
                m_ledBuffer.setLED(i+1,Color.kDarkViolet);
                m_ledBuffer.setLED(i+2,Color.kDarkViolet);
                m_ledBuffer.setLED(i+3,Color.kDarkViolet);
                m_ledBuffer.setLED(i+4,Color.kDarkViolet);
            } else{
                m_ledBuffer.setRGB(i ,255, 65, 0);
                m_ledBuffer.setRGB(i+1 ,255, 65, 0);
                m_ledBuffer.setRGB(i+2 ,255, 65, 0);
                m_ledBuffer.setRGB(i+3 ,255, 65, 0);
                m_ledBuffer.setRGB(i+4 ,255, 65, 0);
            }
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void setWingo(){
        for(int i = 0; (i + 5) < m_ledBuffer.getLength(); i = i + 5){
            if ((i % 10) == 0){
                m_ledBuffer.setLED(i,Color.kDarkViolet);
                m_ledBuffer.setLED(i+1,Color.kDarkViolet);
                m_ledBuffer.setLED(i+2,Color.kDarkViolet);
                m_ledBuffer.setLED(i+3,Color.kDarkViolet);
                m_ledBuffer.setLED(i+4,Color.kDarkViolet);
            } else{
                m_ledBuffer.setLED(i ,Color.kGreen);
                m_ledBuffer.setLED(i+1 ,Color.kGreen);
                m_ledBuffer.setLED(i+2 ,Color.kGreen);
                m_ledBuffer.setLED(i+3 ,Color.kGreen);
                m_ledBuffer.setLED(i+4 ,Color.kGreen);
            }
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void setGatewayColors(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            if ((i % 2) == 0){
                m_ledBuffer.setLED(i,Color.kWhite);
            } else{
                m_ledBuffer.setLED(i,Color.kLightBlue);
            }
            
        }
        m_led.setData(m_ledBuffer);
    }

    public void turnOn(){
        m_ledOn = true;
    }

    public void setOff(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i,0,0,0);
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {

        if ((DriverStation.getMatchTime() <= 20.0 /*130.0*/) && (DriverStation.getMatchType() != DriverStation.MatchType.None) && (m_auto.getAsBoolean() == false)){
            if (DriverStation.getMatchTime() <= 10.0 /*140.0*/){
                setRedWhite();
            } else {
                    setRed();
            }
        }

        if (!m_ledOn){
            setOff();
        }
    }
}
