// (C) 2013 David Feil-Seifer

package edu.yale.dragonbot;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;

import dragon_msgs.*;
/*import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionGoal;
import dragon_msgs.VisemeActionResult;
import dragon_msgs.VisemeFeedback;
import dragon_msgs.VisemeGoal;
import dragon_msgs.VisemeResult;*/
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;


public class DragonbotNode extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    
    final DragonBotComm comm = new DragonBotComm();

    int count =1 ;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
    {
      comm.update();
      count++;
      if(count%10000 ==0)
      comm.sendNetworkDebug(count/10000);
      try { Thread.sleep(50); } catch( Exception e) {}
    }

    final Log log = connectedNode.getLog();
    Subscriber<dragon_msgs.VisemeGoal> subscriber = connectedNode.newSubscriber("dragonbot_viseme", dragon_msgs.VisemeGoal._TYPE);
    subscriber.addMessageListener(new MessageListener<dragon_msgs.VisemeGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.VisemeGoal goal) {
        System.out.println( "setting viseme: " + goal.getConstant() );
        if(goal.getConstant().equalsIgnoreCase("off"))
        {
          comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
        }
        else
        {
          comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
          if(goal.getConstant().equalsIgnoreCase("IDLE"))
            comm.sendViseme(VISEME.IDLE);
          else if(goal.getConstant().equalsIgnoreCase("AA_AH"))
            comm.sendViseme(VISEME.VISEME_AA_AH);
          else if(goal.getConstant().equalsIgnoreCase("AO_AW"))
            comm.sendViseme(VISEME.VISEME_AO_AW);
          else if(goal.getConstant().equalsIgnoreCase("CH_SH_ZH"))
            comm.sendViseme(VISEME.VISEME_CH_SH_ZH);
          else if(goal.getConstant().equalsIgnoreCase("EH_AE_AY"))
            comm.sendViseme(VISEME.VISEME_EH_AE_AY);
          else if(goal.getConstant().equalsIgnoreCase("EY"))
            comm.sendViseme(VISEME.VISEME_EY);
          else if(goal.getConstant().equalsIgnoreCase("L"))
            comm.sendViseme(VISEME.VISEME_L);
          else if(goal.getConstant().equalsIgnoreCase("M_B_P"))
            comm.sendViseme(VISEME.VISEME_M_B_P);
          else if(goal.getConstant().equalsIgnoreCase("N_NG_D_Z"))
            comm.sendViseme(VISEME.VISEME_N_NG_D_Z);
          else if(goal.getConstant().equalsIgnoreCase("R_ER"))
            comm.sendViseme(VISEME.VISEME_R_ER);
          comm.update();
        }
      }
    });

    /******************************/
      // runloop

    connectedNode.executeCancellableLoop( new CancellableLoop() {
      private int sequenceNumber;
      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        System.out.print(".");
        comm.update();
        Thread.sleep(50);
      }
    });

    /******************************/

  }

}
