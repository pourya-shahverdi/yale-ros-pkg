/*
 * dragonbot_driver
 * Copyright (c) 2012, David Feil-Seifer
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the yale-ros-pkgs nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package edu.yale.dragonbot_driver;

import java.io.IOException;

import mcbmini.AndroidIOIOMCBMiniServer;
import mcbmini.DebugMCBMiniServer;
import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniServer;
import mcbmini.MCBMiniServer.MCBMiniBoardDisabledHandler;
import mcbmini.MCBMiniServer.MCBMiniResponseHandler;
import mcbmini.serial.MCBMiniNativeLoader;
import mcbmini.utils.XMLUtils;
import mcbmini.utils.XMLUtils.XMLResults;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import sensor_msgs.JointState;

/**
  pending permission from original author, code will be BSD licensed, but for now all redistribution rights reserved
**/

/**
 * @author siggi, ROS rewrites from David Feil-Seifer <david.feil-seifer@yale.edu>
 * @date Jul 18, 2012, ROS rewrites Aug 2nd, 2012
 */

public class DragonbotDriver extends AbstractNodeMain {
  
  private boolean mDebug;
  private String mConfigFileName;
			
    // TODO: modify to handle multiple boards

      private MCBMiniServer server = null;
			private MCBMiniBoard board;

  private sensor_msgs.JointState mJointMsg;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_driver");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    
    ParameterTree params = connectedNode.getParameterTree();
    mConfigFileName = params.getString("/dragonbot/config_file", "");
    mDebug = params.getBoolean("/dragonbot/debug");

		/*
		 * Read the xml (currently as a hard-coded default file, change to filename passed in as a param)
		 */
		//args = new String[]{"test.xml"};
    XMLResults results = null;
 		try {
    	results = XMLUtils.parseMCBMiniConfigFile( mConfigFileName );
 		} catch (Exception e1) {
     	System.err.println("Can't parse input file: "+e1.getMessage());
  		System.exit(0);
	  }

 		/*
     * Initialize the server with the board list and serial port name from the xml file
 		 */
 		try {
     	// This allows us to not have a connected stack, for debugging purposes
  		if( mDebug ) server = new DebugMCBMiniServer(results.boards);
	  	else server = new MCBMiniServer(results.port_name, results.boards);
       //			server = new AndroidIOIOMCBMiniServer(boards, ioio_rx_pin, ioio_tx_pin)
	  } catch (IOException e) {
      // TODO Auto-generated catch block
		  e.printStackTrace();

      // TODO: double-check to make sure correct behavior
      System.exit(0);
    }

    // declare publisher for motor pos
    final Publisher<sensor_msgs.JointState> joint_pub = 
      connectedNode.newPublisher("joint_state", sensor_msgs.JointState._TYPE);

		/*
		 * Create a loop that loops at 50Hz forever
		 */

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;

		    /*
    		 * Enable the board
		     * We could also change any other parameters if we wanted to
         * TODO: handle mulitple boards
    		 */
		    board = server.getBoards().get(0);
    		board.setChannelBParameter(ChannelParameter.ENABLED, 1);
  
        // TODO: modify to handle multiple boards
        java.util.ArrayList<java.lang.String> names = new java.util.ArrayList<java.lang.String>();
        names.add("0_A");
        names.add("0_B");
        double[] zeros = new double[]{0,0};

        mJointMsg = joint_pub.newMessage();
        mJointMsg.setPosition(zeros);
        mJointMsg.setEffort(zeros);
        mJointMsg.setVelocity(zeros);
        mJointMsg.setName(names);
        

		    /*
    		 * Register a handler for disable events (boards can be disabled on timeouts or fault conditions like overheating)
         * TODO: handle multiple boards
		     */
    		server.addBoardDisabledEventHandler(new MCBMiniBoardDisabledHandler() {
		    	@Override
    			public void handleBoardDisableEvent(MCBMiniBoard board, Channel ch) {
		    		System.out.println("Board notified disable event: "+board.getId()+": "+ch);
    			}
    		});

      }

      @Override
      protected void loop() throws InterruptedException {

        sequenceNumber++;
//        Thread.sleep(1000);
		  	// Update the server, this is important and has to be done in an update loop of the application main thread
			  server.update();
        
        

  			// Create the sinusoidal signal
	  		int sin = (int)( 512 + 500 * Math.sin( (float)sequenceNumber/20f ) );

		  	// Set the target position of the motor to the sinusoidal signal
			  board.setChannelBParameter(ChannelParameter.TARGET_TICK, sin);

  			// publish tick position for motor
	  		System.out.println("Actual tick: " + board.getChannelBParameter(ChannelParameter.CURRENT_TICK) );
        double[] motor_vals = mJointMsg.getPosition();
        motor_vals[0] = board.getChannelAParameter(ChannelParameter.CURRENT_TICK);
        motor_vals[1] = board.getChannelBParameter(ChannelParameter.CURRENT_TICK);
        mJointMsg.setPosition(motor_vals);
        std_msgs.Header header = mJointMsg.getHeader();
        header.setStamp( connectedNode.getCurrentTime() );
        mJointMsg.setHeader(header);
        joint_pub.publish(mJointMsg);        


		  	// Every now and then make a request for a parameter just for fun
			  if( sequenceNumber % 200 == 0 ){
				  server.sendRequestForResponse(board, Channel.B, Command.MOTOR_CURRENT, new MCBMiniResponseHandler() {
					  @Override
  					public void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value) {
	  					System.out.println("Received response to request: "+command+": "+value);
		  			}
			  	});
  			} // if sequenceNUmber

	  		/*
		  	 * Every now and then we print out the update rates of the system (the 2nd one is more interesting, it is the response frequency of the boards
			   * if it is ever very different from our internal one then we might have a comm problem)
  			 */
	  		if( sequenceNumber % 100 == 0 ){
  				float[] fps = server.getUpdateRates(null);
	  			System.out.println("Internal update rate: "+fps[0]+" Hz");
		  		System.out.println("Board cycle update rate: "+fps[1]+" Hz");
			  	board.setChannelBParameter(ChannelParameter.ENABLED, 1);
  			} //if sequenceNumber

	  		// Sleep to set the update frequency
		  	try {
			  	Thread.sleep(20);
  			} catch (InterruptedException e) {
	  			// TODO Auto-generated catch block
		  		e.printStackTrace();
			  } // catch
		  } // loop
    });
  }
}



