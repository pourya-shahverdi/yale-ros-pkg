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
import org.ros.node.topic.Publisher;

/**
  pending permission from original author, code will be BSD licensed, but for now all redistribution rights reserved
**/

/**
 * @author siggi, ROS rewrites from David Feil-Seifer <david.feil-seifer@yale.edu>
 * @date Jul 18, 2012, ROS rewrites Aug 2nd, 2012
 */

public class DragonbotDriver extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_driver");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {

		/*
		 * Read the xml (currently as a hard-coded default file, change to filename passed in as a param)
		 */
		//args = new String[]{"test.xml"};

		/*
		 * Create a loop that loops at 50Hz forever
		 */

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;
			private boolean DEBUG;
			private MCBMiniServer server = null;
			private MCBMiniBoard board;
      @Override
      protected void setup() {
        sequenceNumber = 0;
				DEBUG=false;
		XMLResults results = null;
		try {
			results = XMLUtils.parseMCBMiniConfigFile( "/home/dfseifer/fuerte-ros/yale-ros-pkg/dragonbot_core/dragonbot_driver/config/example_motor_config.xml" );
		} catch (Exception e1) {
			System.err.println("Can't parse input file: "+e1.getMessage());
			System.exit(0);
		}


		/*
		 * Initialize the server with the board list and serial port name from the xml file
		 */
		try {
			// This allows us to not have a connected stack, for debugging purposes
			//if( DEBUG ) server = new DebugMCBMiniServer(results.boards);
			/*else */server = new MCBMiniServer(results.port_name, results.boards);
//			server = new AndroidIOIOMCBMiniServer(boards, ioio_rx_pin, ioio_tx_pin)
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		/*
		 * Enable the board
		 * We could also change any other parameters if we wanted to
		 */
		board = server.getBoards().get(0);
		board.setChannelBParameter(ChannelParameter.ENABLED, 1);

		/*
		 * Register a handler for disable events (boards can be disabled on timeouts or fault conditions like overheating)
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

			// Print out the actual tick position for the motor
			System.out.println("Actual tick: " + board.getChannelBParameter(ChannelParameter.CURRENT_TICK) );

			// Every now and then make a request for a parameter just for fun
			if( sequenceNumber % 200 == 0 ){
				server.sendRequestForResponse(board, Channel.B, Command.MOTOR_CURRENT, new MCBMiniResponseHandler() {
					@Override
					public void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value) {
						System.out.println("Received response to request: "+command+": "+value);
					}
				});
			}

			/*
			 * Every now and then we print out the update rates of the system (the 2nd one is more interesting, it is the response frequency of the boards
			 * if it is ever very different from our internal one then we might have a comm problem)
			 */
			if( sequenceNumber % 100 == 0 ){
				float[] fps = server.getUpdateRates(null);
				System.out.println("Internal update rate: "+fps[0]+" Hz");
				System.out.println("Board cycle update rate: "+fps[1]+" Hz");
				board.setChannelBParameter(ChannelParameter.ENABLED, 1);
			}

			// Sleep to set the update frequency
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}



    });
  }
}



