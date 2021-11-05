package it.sun.fri;

import java.util.Vector;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Creates a FRI Session.
 */
public class LBRJointPositionOverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private int _sendPeriodMilliSec;
    private int _portOnController;
    private int _portOnRemote;
    Vector<String> _clientNames = new Vector<String>();
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        
        // create a possible client list
        _clientNames.add("192.168.3.1");
        _clientNames.add("192.168.2.90");
        _clientNames.add("172.31.1.10");
        
        _sendPeriodMilliSec = 5;
        
        _portOnController = 30200;
        _portOnRemote = 30200;
    }
    
    private void chooseClientName() throws Exception
    {
    	if(_clientNames.size() < 1)
    	{
    		throw new Exception("No client in the code");
    	}
    	if(_clientNames.size() == 1)
    	{
    		_clientName = _clientNames.get(0);
    		return;
    	}
    	int choice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose the Client",_clientNames.toArray(new String[_clientNames.size()]));
    	_clientName = _clientNames.get(choice);
    	getLogger().info("chosen client: " + _clientName);
    }
    
    private void chooseSendPeriod()
    {
    	
    	int choice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose sendPeriodMilliSec", "1", "2", "3", "5", "10", "100");
    	if (choice == 0) {
            getLogger().info("sendPeriod 1ms chosen");
            _sendPeriodMilliSec = 1;
        }
        else if (choice == 1) {
        	getLogger().info("sendPeriod 2ms chosen");
            _sendPeriodMilliSec = 2;
        }
        else if (choice == 2) {
        	getLogger().info("sendPeriod 3ms chosen");
            _sendPeriodMilliSec = 3;
        }
        else if (choice == 3) {
        	getLogger().info("sendPeriod 5ms chosen");
            _sendPeriodMilliSec = 5;
        }
        else if (choice == 4) {
        	getLogger().info("sendPeriod 10ms chosen");
            _sendPeriodMilliSec = 10;
        }
        else if (choice == 5) {
        	getLogger().info("sendPeriod 100ms chosen");
            _sendPeriodMilliSec = 100;
        }
        else {
            getLogger().warn("Invalid choice: setting sendPeriod to " + _sendPeriodMilliSec + "ms");
        }
    }

    @Override
    public void run()
    {  	 
    	
    	try {
			chooseClientName();
		} catch (final Exception e) {
			getLogger().error(e.getLocalizedMessage());
			return;
		}
    	chooseSendPeriod();    	
    	
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(_sendPeriodMilliSec);
        friConfiguration.setPortOnController(_portOnController);
        friConfiguration.setPortOnRemote(_portOnRemote);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());
        
        getLogger().info("PortOnController:" + friConfiguration.getPortOnController());
        getLogger().info("PortOnRemote:" + friConfiguration.getPortOnRemote());

        FRISession friSession = new FRISession(friConfiguration);
        
        IFRISessionListener listener = new IFRISessionListener(){
        	@Override
        	public void onFRIConnectionQualityChanged(
        	FRIChannelInformation friChannelInformation){
	        	getLogger().info("QualityChangedEvent - quality:" +
	        	friChannelInformation.getQuality());
        	}
        	@Override
        	public void onFRISessionStateChanged(
        	FRIChannelInformation friChannelInformation){
	        	getLogger().info("SessionStateChangedEvent - session state:" +
	        	friChannelInformation.getFRISessionState());
        	}
        	};
        friSession.addFRISessionListener(listener);

        // wait until FRI session is ready to switch to command mode
        try
        {
        	getLogger().info("Waiting for connection...");
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch(final TimeoutException e)
        {
        	getLogger().error(e.getLocalizedMessage());
        	FRIChannelInformation channelInfo = friSession.getFRIChannelInformation();
        	getLogger().error("Timeout occured - quality: " + channelInfo.getQuality() 
        			+ " - session state: " + channelInfo.getFRISessionState());
        	friSession.close();
        	return;
        }

        getLogger().info("FRI connection established.");
        
        // TODO: Support more client mode
        ClientCommandMode mode = ClientCommandMode.POSITION;
        getLogger().info("Position control mode chosen");
        
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);
        
        // TODO: Support more control mode
        AbstractMotionControlMode ctrMode = new PositionControlMode();

        try {
            PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
            getLogger().info("Robot is ready for remote control.");
            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        }
        catch(final CommandInvalidException e) {
            getLogger().error("Client has been disconnected.");
        } 
        finally {
        	// done
            friSession.close();
        }

    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final LBRJointPositionOverlay app = new LBRJointPositionOverlay();
        app.runApplication();
    }

}
