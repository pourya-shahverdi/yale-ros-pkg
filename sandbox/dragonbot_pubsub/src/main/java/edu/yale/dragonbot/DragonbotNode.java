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

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    
    final DragonBotComm comm = new DragonBotComm();
    int count =1 ;
/*
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
    {
      comm.update();
      System.out.println( "FPS: " + comm.getFaceDisplayFPS() + " Exp: " + comm.getExpressionCurrent() );
      try { Thread.sleep(33); } catch( Exception e) {}
      count++;
      if(count%10000 ==0) comm.sendNetworkDebug(count/10000);
      System.out.print("?");
    }
*/
    final Log log = connectedNode.getLog();

    /******** Viseme Server ********/

    Subscriber<dragon_msgs.VisemeGoal> viseme_subscriber = connectedNode.newSubscriber("dragonbot_viseme", dragon_msgs.VisemeGoal._TYPE);
    viseme_subscriber.addMessageListener(new MessageListener<dragon_msgs.VisemeGoal>() {
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

    /******** Expression/Motion Server ********/

    Subscriber<dragon_msgs.ExpressionMotionGoal> expression_subscriber = connectedNode.newSubscriber("dragonbot_expression", dragon_msgs.ExpressionMotionGoal._TYPE);
    expression_subscriber.addMessageListener(new MessageListener<dragon_msgs.ExpressionMotionGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.ExpressionMotionGoal goal) {

        if(goal.getType().equalsIgnoreCase("expression"))
        {
          if(goal.getConstant().equalsIgnoreCase("angry"))
            comm.sendExpression(EXPRESSION.EXPRESSION_ANGRY);
          else if(goal.getConstant().equalsIgnoreCase("disgusted"))
            comm.sendExpression(EXPRESSION.EXPRESSION_DISGUSTED);
          else if(goal.getConstant().equalsIgnoreCase("frustrated"))
            comm.sendExpression(EXPRESSION.EXPRESSION_FRUSTRATED);
          else if(goal.getConstant().equalsIgnoreCase("mischievous"))
            comm.sendExpression(EXPRESSION.EXPRESSION_MISCHIEVOUS);
          else if(goal.getConstant().equalsIgnoreCase("shy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SHY);
          else if(goal.getConstant().equalsIgnoreCase("bored")||goal.getConstant().equalsIgnoreCase("bored_unimpressed"))
            comm.sendExpression(EXPRESSION.EXPRESSION_BORED_UNIMPRESSED);
          else if(goal.getConstant().equalsIgnoreCase("ecstatic"))
            comm.sendExpression(EXPRESSION.EXPRESSION_ECSTATIC);
          else if(goal.getConstant().equalsIgnoreCase("happy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_HAPPY);
          else if(goal.getConstant().equalsIgnoreCase("puppy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_PUPPY);
          else if(goal.getConstant().equalsIgnoreCase("surprised"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SURPRISED);
          else if(goal.getConstant().equalsIgnoreCase("confused"))
            comm.sendExpression(EXPRESSION.EXPRESSION_CONFUSED);
          else if(goal.getConstant().equalsIgnoreCase("frightened"))
            comm.sendExpression(EXPRESSION.EXPRESSION_FRIGHTENED);
          else if(goal.getConstant().equalsIgnoreCase("lovestruck"))
            comm.sendExpression(EXPRESSION.EXPRESSION_LOVESTRUCK);
          else if(goal.getConstant().equalsIgnoreCase("sad"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SAD);

          comm.update();
        }
        else if(goal.getType().equalsIgnoreCase("motion"))
        {
          if(goal.getConstant().equalsIgnoreCase("afraid"))
            comm.sendMotion(MOTION.MOTION_AFRAID);
          else if(goal.getConstant().equalsIgnoreCase("blech"))
            comm.sendMotion(MOTION.MOTION_BLECH);
          else if(goal.getConstant().equalsIgnoreCase("farted"))
            comm.sendMotion(MOTION.MOTION_FARTED);
          else if(goal.getConstant().equalsIgnoreCase("idunno"))
            comm.sendMotion(MOTION.MOTION_IDUNNO);
          else if(goal.getConstant().equalsIgnoreCase("interest"))
            comm.sendMotion(MOTION.MOTION_INTEREST);
          else if(goal.getConstant().equalsIgnoreCase("mmhmmm"))
            comm.sendMotion(MOTION.MOTION_MMHMMM);
          else if(goal.getConstant().equalsIgnoreCase("no"))
            comm.sendMotion(MOTION.MOTION_NO);
          else if(goal.getConstant().equalsIgnoreCase("shy"))
            comm.sendMotion(MOTION.MOTION_SHY);
          else if(goal.getConstant().equalsIgnoreCase("think"))
            comm.sendMotion(MOTION.MOTION_THINK);
          else if(goal.getConstant().equalsIgnoreCase("woah"))
            comm.sendMotion(MOTION.MOTION_WOAH);
          else if(goal.getConstant().equalsIgnoreCase("yes"))
            comm.sendMotion(MOTION.MOTION_YES);
          else if(goal.getConstant().equalsIgnoreCase("anticipation"))
            comm.sendMotion(MOTION.MOTION_ANTICIPATION);
          else if(goal.getConstant().equalsIgnoreCase("cheer"))
            comm.sendMotion(MOTION.MOTION_CHEER);
          else if(goal.getConstant().equalsIgnoreCase("heh"))
            comm.sendMotion(MOTION.MOTION_HEH);
          else if(goal.getConstant().equalsIgnoreCase("ilikeit") || goal.getConstant().equalsIgnoreCase("i_like_it"))
            comm.sendMotion(MOTION.MOTION_I_LIKE_IT);
          else if(goal.getConstant().equalsIgnoreCase("laugh") ||goal.getConstant().equalsIgnoreCase("laugh1"))
            comm.sendMotion(MOTION.MOTION_LAUGH1);
          else if(goal.getConstant().equalsIgnoreCase("mph"))
            comm.sendMotion(MOTION.MOTION_MPH);
          else if(goal.getConstant().equalsIgnoreCase("question"))
            comm.sendMotion(MOTION.MOTION_QUESTION);
          else if(goal.getConstant().equalsIgnoreCase("sneeze"))
            comm.sendMotion(MOTION.MOTION_SNEEZE);
          else if(goal.getConstant().equalsIgnoreCase("wakeup"))
            comm.sendMotion(MOTION.MOTION_WAKEUP);
          else if(goal.getConstant().equalsIgnoreCase("yay"))
            comm.sendMotion(MOTION.MOTION_YAY);
          else if(goal.getConstant().equalsIgnoreCase("yummm"))
            comm.sendMotion(MOTION.MOTION_YUMMM);
          else if(goal.getConstant().equalsIgnoreCase("bite"))
            comm.sendMotion(MOTION.MOTION_BITE);
          else if(goal.getConstant().equalsIgnoreCase("crazy_laugh"))
            comm.sendMotion(MOTION.MOTION_CRAZY_LAUGH);
          else if(goal.getConstant().equalsIgnoreCase("hungry"))
            comm.sendMotion(MOTION.MOTION_HUNGRY);
          else if(goal.getConstant().equalsIgnoreCase("iwantit") ||goal.getConstant().equalsIgnoreCase("i_want_it"))
            comm.sendMotion(MOTION.MOTION_I_WANT_IT);
          else if(goal.getConstant().equalsIgnoreCase("meh"))
            comm.sendMotion(MOTION.MOTION_MEH);
          else if(goal.getConstant().equalsIgnoreCase("nah_nah") || goal.getConstant().equalsIgnoreCase("nahnah"))
            comm.sendMotion(MOTION.MOTION_NAH_NAH);
          else if(goal.getConstant().equalsIgnoreCase("sad"))
            comm.sendMotion(MOTION.MOTION_SAD);
          else if(goal.getConstant().equalsIgnoreCase("surprise"))
            comm.sendMotion(MOTION.MOTION_SURPRISE);
          else if(goal.getConstant().equalsIgnoreCase("weee"))
            comm.sendMotion(MOTION.MOTION_WEEE);
          else if(goal.getConstant().equalsIgnoreCase("yawn"))
            comm.sendMotion(MOTION.MOTION_YAWN);
        }
      }
    });
    /******** IK Server ********/
    Subscriber<dragon_msgs.IKGoal> ik_subscriber = connectedNode.newSubscriber("dragonbot_ik", dragon_msgs.IKGoal._TYPE);
    ik_subscriber.addMessageListener(new MessageListener<dragon_msgs.IKGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.IKGoal goal) {
      }
    });
   /******** LookAt Server ********/
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
      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {

        comm.update();

        comm.getMotionCurrent();
        comm.getExpressionCurrent();
        comm.getIKCurrent();
        comm.getLookatTargetCurrent();
        comm.getVisemeTargetCurrent();
        comm.getPoseTargetCurrent();

        Thread.sleep(33);
        //if(sequenceNumber%10000 ==0) comm.sendNetworkDebug(sequenceNumber/10000);
        if( Float.isNaN( comm.getFaceDisplayFPS() ) )
          System.out.print("-");
        else
          System.out.print("+");

        sequenceNumber++;
      }
    });

    /******************************/

  }

}
