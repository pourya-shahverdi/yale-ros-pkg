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

  private DragonBotComm comm = null;
  private String visemeTarget = null;

  void set_viseme( String visemeTarget )
  {
        if(visemeTarget.equalsIgnoreCase("off"))
        {
          comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
        }
        else
        {
          comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
          if(visemeTarget.equalsIgnoreCase("IDLE"))
            comm.sendViseme(VISEME.IDLE);
          else if(visemeTarget.equalsIgnoreCase("AA_AH"))
            comm.sendViseme(VISEME.VISEME_AA_AH);
          else if(visemeTarget.equalsIgnoreCase("AO_AW"))
            comm.sendViseme(VISEME.VISEME_AO_AW);
          else if(visemeTarget.equalsIgnoreCase("CH_SH_ZH"))
            comm.sendViseme(VISEME.VISEME_CH_SH_ZH);
          else if(visemeTarget.equalsIgnoreCase("EH_AE_AY"))
            comm.sendViseme(VISEME.VISEME_EH_AE_AY);
          else if(visemeTarget.equalsIgnoreCase("EY"))
            comm.sendViseme(VISEME.VISEME_EY);
          else if(visemeTarget.equalsIgnoreCase("L"))
            comm.sendViseme(VISEME.VISEME_L);
          else if(visemeTarget.equalsIgnoreCase("M_B_P"))
            comm.sendViseme(VISEME.VISEME_M_B_P);
          else if(visemeTarget.equalsIgnoreCase("N_NG_D_Z"))
            comm.sendViseme(VISEME.VISEME_N_NG_D_Z);
          else if(visemeTarget.equalsIgnoreCase("R_ER"))
            comm.sendViseme(VISEME.VISEME_R_ER);
          comm.update();
        }

  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    

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
        System.out.println( "setting viseme: " + visemeTarget );
        visemeTarget = goal.getConstant();
      }
    });

    /******** Expression/Motion Server ******** /

    Subscriber<dragon_msgs.ExpressionMotionGoal> expression_subscriber = connectedNode.newSubscriber("dragonbot_expression", dragon_msgs.ExpressionMotionGoal._TYPE);
    expression_subscriber.addMessageListener(new MessageListener<dragon_msgs.ExpressionMotionGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.ExpressionMotionGoal goal) {

        if(goal.getType().equalsIgnoreCase("expression"))
        {
          if(visemeTarget.equalsIgnoreCase("angry"))
            comm.sendExpression(EXPRESSION.EXPRESSION_ANGRY);
          else if(visemeTarget.equalsIgnoreCase("disgusted"))
            comm.sendExpression(EXPRESSION.EXPRESSION_DISGUSTED);
          else if(visemeTarget.equalsIgnoreCase("frustrated"))
            comm.sendExpression(EXPRESSION.EXPRESSION_FRUSTRATED);
          else if(visemeTarget.equalsIgnoreCase("mischievous"))
            comm.sendExpression(EXPRESSION.EXPRESSION_MISCHIEVOUS);
          else if(visemeTarget.equalsIgnoreCase("shy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SHY);
          else if(visemeTarget.equalsIgnoreCase("bored")||goal.getConstant().equalsIgnoreCase("bored_unimpressed"))
            comm.sendExpression(EXPRESSION.EXPRESSION_BORED_UNIMPRESSED);
          else if(visemeTarget.equalsIgnoreCase("ecstatic"))
            comm.sendExpression(EXPRESSION.EXPRESSION_ECSTATIC);
          else if(visemeTarget.equalsIgnoreCase("happy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_HAPPY);
          else if(visemeTarget.equalsIgnoreCase("puppy"))
            comm.sendExpression(EXPRESSION.EXPRESSION_PUPPY);
          else if(visemeTarget.equalsIgnoreCase("surprised"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SURPRISED);
          else if(visemeTarget.equalsIgnoreCase("confused"))
            comm.sendExpression(EXPRESSION.EXPRESSION_CONFUSED);
          else if(visemeTarget.equalsIgnoreCase("frightened"))
            comm.sendExpression(EXPRESSION.EXPRESSION_FRIGHTENED);
          else if(visemeTarget.equalsIgnoreCase("lovestruck"))
            comm.sendExpression(EXPRESSION.EXPRESSION_LOVESTRUCK);
          else if(visemeTarget.equalsIgnoreCase("sad"))
            comm.sendExpression(EXPRESSION.EXPRESSION_SAD);

          comm.update();
        }
        else if(goal.getType().equalsIgnoreCase("motion"))
        {
          if(visemeTarget.equalsIgnoreCase("afraid"))
            comm.sendMotion(MOTION.MOTION_AFRAID);
          else if(visemeTarget.equalsIgnoreCase("blech"))
            comm.sendMotion(MOTION.MOTION_BLECH);
          else if(visemeTarget.equalsIgnoreCase("farted"))
            comm.sendMotion(MOTION.MOTION_FARTED);
          else if(visemeTarget.equalsIgnoreCase("idunno"))
            comm.sendMotion(MOTION.MOTION_IDUNNO);
          else if(visemeTarget.equalsIgnoreCase("interest"))
            comm.sendMotion(MOTION.MOTION_INTEREST);
          else if(visemeTarget.equalsIgnoreCase("mmhmmm"))
            comm.sendMotion(MOTION.MOTION_MMHMMM);
          else if(visemeTarget.equalsIgnoreCase("no"))
            comm.sendMotion(MOTION.MOTION_NO);
          else if(visemeTarget.equalsIgnoreCase("shy"))
            comm.sendMotion(MOTION.MOTION_SHY);
          else if(visemeTarget.equalsIgnoreCase("think"))
            comm.sendMotion(MOTION.MOTION_THINK);
          else if(visemeTarget.equalsIgnoreCase("woah"))
            comm.sendMotion(MOTION.MOTION_WOAH);
          else if(visemeTarget.equalsIgnoreCase("yes"))
            comm.sendMotion(MOTION.MOTION_YES);
          else if(visemeTarget.equalsIgnoreCase("anticipation"))
            comm.sendMotion(MOTION.MOTION_ANTICIPATION);
          else if(visemeTarget.equalsIgnoreCase("cheer"))
            comm.sendMotion(MOTION.MOTION_CHEER);
          else if(visemeTarget.equalsIgnoreCase("heh"))
            comm.sendMotion(MOTION.MOTION_HEH);
          else if(visemeTarget.equalsIgnoreCase("ilikeit") || goal.getConstant().equalsIgnoreCase("i_like_it"))
            comm.sendMotion(MOTION.MOTION_I_LIKE_IT);
          else if(visemeTarget.equalsIgnoreCase("laugh") ||goal.getConstant().equalsIgnoreCase("laugh1"))
            comm.sendMotion(MOTION.MOTION_LAUGH1);
          else if(visemeTarget.equalsIgnoreCase("mph"))
            comm.sendMotion(MOTION.MOTION_MPH);
          else if(visemeTarget.equalsIgnoreCase("question"))
            comm.sendMotion(MOTION.MOTION_QUESTION);
          else if(visemeTarget.equalsIgnoreCase("sneeze"))
            comm.sendMotion(MOTION.MOTION_SNEEZE);
          else if(visemeTarget.equalsIgnoreCase("wakeup"))
            comm.sendMotion(MOTION.MOTION_WAKEUP);
          else if(visemeTarget.equalsIgnoreCase("yay"))
            comm.sendMotion(MOTION.MOTION_YAY);
          else if(visemeTarget.equalsIgnoreCase("yummm"))
            comm.sendMotion(MOTION.MOTION_YUMMM);
          else if(visemeTarget.equalsIgnoreCase("bite"))
            comm.sendMotion(MOTION.MOTION_BITE);
          else if(visemeTarget.equalsIgnoreCase("crazy_laugh"))
            comm.sendMotion(MOTION.MOTION_CRAZY_LAUGH);
          else if(visemeTarget.equalsIgnoreCase("hungry"))
            comm.sendMotion(MOTION.MOTION_HUNGRY);
          else if(visemeTarget.equalsIgnoreCase("iwantit") ||goal.getConstant().equalsIgnoreCase("i_want_it"))
            comm.sendMotion(MOTION.MOTION_I_WANT_IT);
          else if(visemeTarget.equalsIgnoreCase("meh"))
            comm.sendMotion(MOTION.MOTION_MEH);
          else if(visemeTarget.equalsIgnoreCase("nah_nah") || goal.getConstant().equalsIgnoreCase("nahnah"))
            comm.sendMotion(MOTION.MOTION_NAH_NAH);
          else if(visemeTarget.equalsIgnoreCase("sad"))
            comm.sendMotion(MOTION.MOTION_SAD);
          else if(visemeTarget.equalsIgnoreCase("surprise"))
            comm.sendMotion(MOTION.MOTION_SURPRISE);
          else if(visemeTarget.equalsIgnoreCase("weee"))
            comm.sendMotion(MOTION.MOTION_WEEE);
          else if(visemeTarget.equalsIgnoreCase("yawn"))
            comm.sendMotion(MOTION.MOTION_YAWN);
        }
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
      @Override
      protected void setup() {
        comm = new DragonBotComm();
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {

        comm.update();
        if( visemeTarget != null )
          set_viseme(visemeTarget);

        comm.getMotionCurrent();
        comm.getExpressionCurrent();
        comm.getIKCurrent();
        comm.getLookatTargetCurrent();
        comm.getVisemeTargetCurrent();

        comm.getPoseTargetCurrent();

        Thread.sleep(33);
        if(sequenceNumber%100 ==0) comm.sendNetworkDebug(sequenceNumber/100);
        if( Float.isNaN( comm.getFaceDisplayFPS() ) ) {
          System.out.print("-");
          sequenceNumber++;
        }
        else
          System.out.print("+");


      }
    });

    /******************************/

  }

}
