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
import prg.content.dragonbot.teleopstandalone.DragonBotComm;

import prg.content.dragonbot.teleopstandalone.Constants.*;


public class DragonbotNode extends AbstractNodeMain {

  VISEME visemeTarget = VISEME.IDLE;
  EXPRESSION expressionTarget;
  void set_viseme( String goal )
  {
    if(goal.equalsIgnoreCase("off"))
    {

      visemeTarget = VISEME.IDLE;
    }
    else
    {

      if(goal.equalsIgnoreCase("IDLE"))
        visemeTarget=(VISEME.IDLE);
      else if(goal.equalsIgnoreCase("AA_AH"))
        visemeTarget=(VISEME.VISEME_AA_AH);
      else if(goal.equalsIgnoreCase("AO_AW"))
        visemeTarget=(VISEME.VISEME_AO_AW);
      else if(goal.equalsIgnoreCase("CH_SH_ZH"))
        visemeTarget=(VISEME.VISEME_CH_SH_ZH);
      else if(goal.equalsIgnoreCase("EH_AE_AY"))
        visemeTarget=(VISEME.VISEME_EH_AE_AY);
      else if(goal.equalsIgnoreCase("EY"))
        visemeTarget=(VISEME.VISEME_EY);
      else if(goal.equalsIgnoreCase("L"))
        visemeTarget=(VISEME.VISEME_L);
      else if(goal.equalsIgnoreCase("M_B_P"))
        visemeTarget=(VISEME.VISEME_M_B_P);
      else if(goal.equalsIgnoreCase("N_NG_D_Z"))
        visemeTarget=(VISEME.VISEME_N_NG_D_Z);
      else if(goal.equalsIgnoreCase("R_ER"))
        visemeTarget=(VISEME.VISEME_R_ER);
    }
  }

  void set_expression( String expressionTarget )
  {

  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    

    int count =1 ;
    final Log log = connectedNode.getLog();

    /******** Viseme Server ********/

    Subscriber<dragon_msgs.VisemeGoal> viseme_subscriber = connectedNode.newSubscriber("dragonbot_viseme", dragon_msgs.VisemeGoal._TYPE);
    viseme_subscriber.addMessageListener(new MessageListener<dragon_msgs.VisemeGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.VisemeGoal goal) {
        System.out.println( "setting viseme: " + goal.getConstant() );
        set_viseme(goal.getConstant());
      }
    });

    /******** Expression/Motion Server ********/

    Subscriber<dragon_msgs.ExpressionMotionGoal> expression_subscriber = connectedNode.newSubscriber("dragonbot_expression", dragon_msgs.ExpressionMotionGoal._TYPE);
    expression_subscriber.addMessageListener(new MessageListener<dragon_msgs.ExpressionMotionGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.ExpressionMotionGoal goal) {
        if( goal.getType().equalsIgnoreCase("expression") )
          set_expression(goal.getConstant());
      }
    });
    /******** IK Server ******** /
    Subscriber<dragon_msgs.IKGoal> ik_subscriber = connectedNode.newSubscriber("dragonbot_ik", dragon_msgs.IKGoal._TYPE);
    ik_subscriber.addMessageListener(new MessageListener<dragon_msgs.IKGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.IKGoal goal) {
      }
    });
   /******** LookAt Server ******** /
    Subscriber<dragon_msgs.LookatGoal> lookat_subscriber = connectedNode.newSubscriber("dragonbot_lookat", dragon_msgs.LookatGoal._TYPE);
    lookat_subscriber.addMessageListener(new MessageListener<dragon_msgs.LookatGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.LookatGoal goal) {
      }
    });


    /******************************/
      // runloop

    connectedNode.executeCancellableLoop( new CancellableLoop() {
      private int sequenceNumber;
      private DragonBotComm comm = null;
      private boolean viseme_set = true;
      private boolean expression_set = true;
      @Override
      protected void setup() {
        comm = new DragonBotComm();
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        try {
        Thread.sleep(33);
        comm.update();
        if(sequenceNumber%100 ==0) comm.sendNetworkDebug(sequenceNumber/100);
        if( Float.isNaN( comm.getFaceDisplayFPS() ) ) {
          System.out.print("-");
          sequenceNumber++;
          return;
        }
        else
          System.out.print("+");
        

        if( visemeTarget == VISEME.IDLE  && !viseme_set ) {
          comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
          comm.sendViseme(visemeTarget);
          viseme_set = true;
        }
        else {
          comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
          viseme_set = false;
        }
          
          
        if( expression_set )
          comm.sendExpression(expressionTarget);


        comm.getMotionCurrent();
        comm.getExpressionCurrent();
        comm.getIKCurrent();
        comm.getLookatTargetCurrent();
        comm.getVisemeTargetCurrent();

        comm.getPoseTargetCurrent();


        }
        catch (Exception e) {
          System.out.println( "Exception: " + e.toString() );
        }

      }
    });

    /******************************/

  }

}
