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
import java.util.List;
import java.util.Map;
import java.util.HashMap;

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
import org.ros.node.parameter.*;
import org.ros.node.topic.*;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;

//for second XML parsing pass
import java.io.File;
import org.w3c.dom.Document;
import org.w3c.dom.*;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException; 

import sensor_msgs.JointState;
import std_msgs.Int32;

/**
  pending permission from original author, code will be BSD licensed, but for now all redistribution rights reserved
**/

/**
 * @author siggi, ROS rewrites from David Feil-Seifer <david.feil-seifer@yale.edu>
 * @date Jul 18, 2012, ROS rewrites Aug 2nd, 2012
 */

public class DragonbotDriver extends AbstractNodeMain {
  
  // TODO: read from config param
  //private double GEAR_RATIO = 50;  
  private boolean mDebug;
  protected String mConfigFileName;

  private int mMotorState = 0;

  // DONE: modify to handle multiple boards

  // needs to be protected to be inhereted from AndroidDriver class
  protected MCBMiniServer server = null;
  private java.util.ArrayList<MCBMiniBoard> boards;

  private Map<String, Float> extraParams;
  private final String MIN_TICKS = "_MIN_TICKS";
  private final String MAX_TICKS = "_MAX_TICKS";
  private final String TICKS_PER_UNIT = "_TICKS_PER_UNIT";

  private sensor_msgs.JointState mJointMsg;
  private sensor_msgs.JointState mJointCmd;

  protected XMLResults results = null;

  protected int getId( java.util.ArrayList<MCBMiniBoard> boards, int set_id )
  {
    for( int i = 0; i < boards.size(); i++ )
    {
      if( boards.get(i).getId() == set_id ) return i;
    }
    return -1;
  }

  // uses regular file system, override for android
  protected int parseConfigFile( String filename )
  {
 		try {
    	results = XMLUtils.parseMCBMiniConfigFile( filename );
 		} catch (Exception e1) {
     	System.err.println("Can't parse input file: " + e1.toString());
  		System.exit(0);
	  }

    // parse for extraParams
    parseForExtraParams( filename );

    return 0;
  }

  protected int parseForExtraParams (String filename)
  {
    try {
      extraParams = new HashMap<String, Float>();
 
      File fXmlFile = new File(filename);
      DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
      DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
      Document doc = dBuilder.parse(fXmlFile);
      doc.getDocumentElement().normalize();
   
      System.out.println("Root element :" + doc.getDocumentElement().getNodeName());
      NodeList nList = doc.getElementsByTagName("MiniBoard");
      System.out.println("-----------------------");
   
      for (int temp = 0; temp < nList.getLength(); temp++) {
   
         Node nNode = nList.item(temp);
         if (nNode.getNodeType() == Node.ELEMENT_NODE) {
            Element eElement = (Element) nNode;
            String id = getTagValue("id", eElement);
            System.out.println("id : " + getTagValue("id", eElement));
            NodeList nList2 = eElement.getElementsByTagName("Motor");

            for (int temp2 = 0; temp2 < nList2.getLength(); temp2++) {
               Node nNode2 = nList2.item(temp2);
               if (nNode2.getNodeType() == Node.ELEMENT_NODE) {
                  Element eElement2 = (Element) nNode2;
                  String channel = getTagValue("channel",eElement2);
                  String min_ticks = getTagValue("min_ticks",eElement2);
                  String max_ticks = getTagValue("max_ticks",eElement2);
                  String ticks_per_unit = getTagValue("ticks_per_unit",eElement2);

                  extraParams.put("abc",new Float("1.0"));
                  extraParams.put(new String(id+channel+MIN_TICKS),new Float(min_ticks));
                  extraParams.put(new String(id+channel+MAX_TICKS),new Float(max_ticks));
                  extraParams.put(new String(id+channel+TICKS_PER_UNIT),new Float(ticks_per_unit));
                  System.out.println( "min ticks: " + new String(id+channel+MIN_TICKS) );
          
                  
                  System.out.println("channel : " + getTagValue("channel", eElement2));
                  System.out.println("max_ticks : " + getTagValue("max_ticks", eElement2));
                  System.out.println("min_ticks : " + getTagValue("min_ticks", eElement2));
                  System.out.println("ticks_per_unit : " + getTagValue("ticks_per_unit", eElement2));
                  
               }
            }
         }
      }
    } catch (Exception e) {
      System.err.println( "things went wrong with lookup stuff" );
      e.printStackTrace();
    }

    return -1;
  }
  
  private static String getTagValue(String sTag, Element eElement) {
    NodeList nlList = eElement.getElementsByTagName(sTag).item(0).getChildNodes();
    Node nValue = (Node) nlList.item(0);
    return nValue.getNodeValue();
  }


  // uses tty Serial, override for Android
  protected int initServer()
  {
 		try {
     	// This allows us to not have a connected stack, for debugging purposes
  		if( mDebug ) server = new DebugMCBMiniServer(results.boards);
	  	else server = new MCBMiniServer("/dev/ttyUSB0", results.boards);
	  } catch (IOException e) {
      // TODO Auto-generated catch block
		  System.err.println( "error opening MCBMini server: " + e.toString() );

      // TODO: double-check to make sure correct behavior
      System.exit(0);
    }
    return 0;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_driver");
  }

  private double radiansToTicks (String id, String channel, double radians)
  {
      double coefficient = 1.0;
      if (!extraParams.containsKey(id+channel+TICKS_PER_UNIT))
      {
          //System.err.println("Could not access TICKS_PER_UNIT");
          //System.exit(0);
      }
      else
      {
        coefficient = extraParams.get(id+channel+TICKS_PER_UNIT).floatValue();
      }
      double ticks = radians*coefficient;
      //System.out.println("Radians:"+radians+"-> Ticks:"+ticks);
      return ticks;
  }

  private double ticksToRadians (String id, String channel, double ticks)
  {
    double coefficient = 1.0;
      if (!extraParams.containsKey(id+channel+TICKS_PER_UNIT))
      {
          //System.err.println("Could not access TICKS_PER_UNIT");
          //System.exit(0);
      }
      else {
        coefficient = extraParams.get(id+channel+TICKS_PER_UNIT).floatValue();
      }
      double radians = ticks/coefficient;
      //System.out.println("Ticks:"+ticks+"-> Radians:"+radians);
      return radians;
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    
    ParameterTree params = connectedNode.getParameterTree();

    mConfigFileName = params.getString("/dragonbot/config_file", "");
    mDebug = params.getBoolean("/dragonbot/debug");

		/*
		 * Read the xml (currently as a hard-coded default file, change to filename passed in as a param)
		 */
    if( parseConfigFile(mConfigFileName) != 0 )
    {
      System.out.println( "uh-oh" );
      System.exit(0);
    }

    System.out.println( "motor config file read successfully" );

 		/*
     * Initialize the server with the board list and serial port name from the xml file
 		 */
    if( initServer() != 0 )
    {
      System.out.println( "uh-oh" );
      System.exit(0);
    }
    
    System.out.println( "MCBMini board(s) initialized successfully" );
    
    // initialize boards data structure from server parse of config file
    boards = new java.util.ArrayList<MCBMiniBoard>(server.getBoards());
    /*
     * Server update and identify default values for PID gains
     * Set dynamic reconfigure params for PID gains 
     */

    //Example listener string /0_A_pgain
    for (int i=0;i<boards.size();i++){
      final int i_f = boards.get(i).getId();
      final char [] boardLetters = {'A','B'};
      for (int j = 0; j<boardLetters.length;j++){
        final int j_f = j;
        //The following two arrays must be of the same length
        final String [] parameterStrings = {"pgain","igain","dgain"};
        final mcbmini.MCBMiniConstants.ChannelParameter[] parameterConstants = {ChannelParameter.P_GAIN, ChannelParameter.I_GAIN, ChannelParameter.D_GAIN};
        for (int k=0; k<parameterStrings.length; k++){
          final int k_f = k;
          String listenerString = "/"+i_f+"_"+boardLetters[j_f]+"_"+parameterStrings[k];
          params.addParameterListener(listenerString, new ParameterListener() {
            @Override
            public void onNewValue(Object value) {
              int chg_val = Integer.parseInt(value.toString());
              System.out.println( "param changed: " + chg_val );
              if (boardLetters[j_f]=='A')
                boards.get(i_f).setChannelAParameter(parameterConstants[k_f], chg_val);
              if (boardLetters[j_f]=='B')
                boards.get(i_f).setChannelBParameter(parameterConstants[k_f], chg_val);
            }
          });
        }
      }
    }

    /*
     * ROS Node publisher and subscriber declarations
     */

    // declare publisher for motor pos
    final Publisher<sensor_msgs.JointState> joint_pub = 
      connectedNode.newPublisher("joint_state", sensor_msgs.JointState._TYPE);

    // declare publisher for motor state
    final Publisher<std_msgs.Int32> motor_state_pub = 
      connectedNode.newPublisher("motor_state", std_msgs.Int32._TYPE);

    // initialize message to have proper names
    // DONE: make work for multiple boards

    java.util.ArrayList<java.lang.String> names = new java.util.ArrayList<java.lang.String>();
    double[] zeros = new double[boards.size()*2];
    
    for( int i = 0; i < boards.size(); i++ )
    {
      System.out.println ("name board ID: " + boards.get(i).getId() );
      names.add(boards.get(i).getId()+"_A");
      names.add(boards.get(i).getId()+"_B");
      zeros[i*2] = 0;
      zeros[i*2+1] = 0;
    }

    mJointCmd=joint_pub.newMessage();
    mJointCmd.setPosition(zeros);
    mJointCmd.setEffort(zeros);
    mJointCmd.setVelocity(zeros);
    mJointCmd.setName(names);
    

    // declare subscriber for motor pos
    Subscriber<sensor_msgs.JointState> subscriber = connectedNode.newSubscriber("cmd_pos", sensor_msgs.JointState._TYPE);
    // handler that is run whenever a joint state command is received
    subscriber.addMessageListener(new MessageListener<sensor_msgs.JointState>() {
      @Override
      public void onNewMessage(sensor_msgs.JointState cmd) {
        // copy data from cmd into mJointCmd
        java.util.ArrayList<java.lang.String> cnames = new java.util.ArrayList<java.lang.String>( cmd.getName() );
        double[] motor_vals = cmd.getPosition();

        //look through each board
        for (int b = 0; b < boards.size(); b++){
          java.util.ArrayList<java.lang.String> pnames = new java.util.ArrayList<java.lang.String>( mJointCmd.getName() );
          double[] cmd_vals = mJointCmd.getPosition();

          for( int i = 0; i < cnames.size(); i++ ) {
            System.out.println( "cnames: " + cnames.get(i) );
            for( int j = 0; j < pnames.size(); j++ ) {
              if( pnames.get(j).equals(cnames.get(i)) ) {
                //copy val into motor array
                String id = ""+cnames.get(i).charAt(0);
                String channel = ""+cnames.get(i).charAt(2);

                double ticks = radiansToTicks(id, channel, motor_vals[i]);
               
                System.out.println( "lookup: " + id+channel+MIN_TICKS );
                double min = extraParams.get(id+channel+MIN_TICKS).floatValue();
                double max = extraParams.get(id+channel+MAX_TICKS).floatValue();

                if (ticks<min) ticks=min;
                if (ticks>max) ticks=max;

                cmd_vals[j] = ticks /* * GEAR_RATIO */;
                System.out.println( "setting motor: " + cnames.get(i) + " to: " + cmd_vals[j] );
                connectedNode.getLog().info( "setting motor: " + cnames.get(i) + " to: " + cmd_vals[j] );
              }
            }
          }
        }
      }
    });

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
         * DONE: handle mulitple boards
    		 */
         
        for( int i = 0; i < boards.size(); i++ )
        {
      		boards.get(i).setChannelAParameter(ChannelParameter.ENABLED, 0);
      		boards.get(i).setChannelBParameter(ChannelParameter.ENABLED, 0);
        }
        // TODO: modify to handle multiple boards
        java.util.ArrayList<java.lang.String> names = new java.util.ArrayList<java.lang.String>();
        double[] zeros = new double[boards.size()*2];
    
        for( int i = 0; i < boards.size(); i++ )
        {
          names.add(boards.get(i).getId()+"_A");
          names.add(boards.get(i).getId()+"_B");
          zeros[i*2] = 0;
          zeros[i*2+1] = 0;
        }

        mJointMsg=joint_pub.newMessage();
        mJointMsg.setPosition(zeros);
        mJointMsg.setEffort(zeros);
        mJointMsg.setVelocity(zeros);
        mJointMsg.setName(names);
        

		    /*
    		 * Register a handler for disable events (boards can be disabled on timeouts or fault conditions like overheating)
         * DONE: handle multiple boards
		     */

        Subscriber<std_msgs.Int32> cmd_motor_state_subscriber = connectedNode.newSubscriber("cmd_motor_state", std_msgs.Int32._TYPE);
        cmd_motor_state_subscriber.addMessageListener(new MessageListener<std_msgs.Int32>() {
          @Override
          public void onNewMessage(std_msgs.Int32 cmd_state) {
            // DONE: make work for all boards
            connectedNode.getLog().info( "setting cmd_motor_state: " + cmd_state.getData() );
            for( int i = 0; i < boards.size(); i++ )
            {
              mMotorState = cmd_state.getData();
              boards.get(i).setChannelAParameter(ChannelParameter.ENABLED, mMotorState);
              boards.get(i).setChannelBParameter(ChannelParameter.ENABLED, mMotorState);
            }

          }
        });

    		server.addBoardDisabledEventHandler(new MCBMiniBoardDisabledHandler() {
		    	@Override
    			public void handleBoardDisableEvent(MCBMiniBoard board, Channel ch) {
		    		connectedNode.getLog().info("Board notified disable event: "+board.getId()+": "+ch);
    			}
    		});

      }

      @Override
      protected void loop() throws InterruptedException {

        //DEBUG: Print PID values
        for (int i=0;i<boards.size();i++){
          final char [] boardLetters = {'A','B'};
          for (int j = 0; j<boardLetters.length;j++){
            //The following two arrays must be of the same length
            final String [] parameterStrings = {"pgain","igain","dgain"};
            final mcbmini.MCBMiniConstants.ChannelParameter[] parameterConstants = {ChannelParameter.P_GAIN, ChannelParameter.I_GAIN, ChannelParameter.D_GAIN};
            for (int k=0; k<parameterStrings.length; k++){
                  //System.out.print(i+"_"+boardLetters[j]+"_"+parameterStrings[k]+":");
                  int val = 0;
                  if (boardLetters[j]=='A')
                    val = boards.get(i).getChannelAParameter(parameterConstants[k]);
                  if (boardLetters[j]=='B')
                    val = boards.get(i).getChannelBParameter(parameterConstants[k]);
                  //System.out.println(val);
            }
          }
        }
        //System.out.println("--------");

        sequenceNumber++;
		  	// Update the server, this is important and has to be done in an update loop of the application main thread
        server.update();
        
		  	// Set the target position of the motor to the commanded tick
        java.util.ArrayList<java.lang.String> cmd_names = new java.util.ArrayList<java.lang.String>(mJointCmd.getName());
        double[] cmd_vals = mJointCmd.getPosition();
        for( int i = 0; i < cmd_names.size(); i++ )
        {
          // DONE: make work for muiltiple boards
          char motor_id = cmd_names.get(i).charAt(2);
          int set_id = Integer.parseInt( cmd_names.get(i).substring(0,1) );
          int board_id = getId(boards, set_id);
          connectedNode.getLog().debug( "setting: "+motor_id+","+board_id );
          if( board_id >= 0 && board_id < boards.size() )
          {
            if( motor_id == 'A' )
              boards.get(board_id).setChannelAParameter(ChannelParameter.TARGET_TICK, (int) cmd_vals[i]);
            else if( motor_id == 'B' )
              boards.get(board_id).setChannelBParameter(ChannelParameter.TARGET_TICK, (int) cmd_vals[i]);
          }
        }


  			// publish tick position for motor
        // DONE: make work for multiple boards
        double[] motor_vals = mJointMsg.getPosition();
        for( int i = 0; i < boards.size(); i++ )
        {
          motor_vals[2*i+0] = ticksToRadians(new String(""+(boards.get(i).getId())), "A", boards.get(i).getChannelAParameter(ChannelParameter.CURRENT_TICK) );
          motor_vals[2*i+1] = ticksToRadians(new String(""+(boards.get(i).getId())), "B", boards.get(i).getChannelBParameter(ChannelParameter.CURRENT_TICK) );
        }
        mJointMsg.setPosition(motor_vals);
        std_msgs.Header header = mJointMsg.getHeader();
        header.setStamp( connectedNode.getCurrentTime() );
        mJointMsg.setHeader(header);
        joint_pub.publish(mJointMsg);        

        std_msgs.Int32 motor_state_msg = motor_state_pub.newMessage();
        motor_state_msg.setData(mMotorState);
        motor_state_pub.publish(motor_state_msg);

	  		/*
		  	 * Every now and then we print out the update rates of the system (the 2nd one is more interesting, it is the response frequency of the boards
			   * if it is ever very different from our internal one then we might have a comm problem)
  			 */
	  		if( sequenceNumber % 100 == 0 ){
  				float[] fps = server.getUpdateRates(null);
	  			connectedNode.getLog().debug("Internal update rate: "+fps[0]+" Hz");
		  		connectedNode.getLog().debug("Board cycle update rate: "+fps[1]+" Hz");
			  	//board.setChannelBParameter(ChannelParameter.ENABLED, 1);
  			} //if sequenceNumber

	  		// Sleep to set the update frequency
		  	try {
			  	Thread.sleep(20);
  			} catch (InterruptedException e) {
	  			// TODO Auto-generated catch block
		  	  //e.printStackTrace();
			  } // catch
		  } // loop
    });
  }
}



