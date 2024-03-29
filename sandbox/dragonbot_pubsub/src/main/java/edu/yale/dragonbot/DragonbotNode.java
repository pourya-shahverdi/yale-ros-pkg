// (C) 2013 David Feil-Seifer

package edu.yale.dragonbot;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;

import dragon_msgs.*;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;

import prg.content.dragonbot.teleopstandalone.Constants.*;


public class DragonbotNode extends AbstractNodeMain {

  VISEME visemeTarget = VISEME.IDLE;
  EXPRESSION expressionTarget;
  MOTION motionTarget;
  EYES blinkTarget = EYES.BLINK_HOLD_STOP;

  private String expressionString;
  private String motionString;
  private String lookatString;
  private float[] lookatTarget;
  
  private String ikString;
  private float[] ikTarget;
  private float[] ikAcc;

  private boolean viseme_set = true;
  private boolean expression_set = true;
  private boolean motion_set = true;
  private boolean lookat_set = true;
  private boolean ik_set = true;
  private boolean ik_param_set = true;
  private boolean blink_set = true;

  void set_lookat( String s, float X, float Y, float Z )
  {
    System.out.println( s + "/" + X + "/" + Y + "/" + Z );
    lookatString = s;
    lookatTarget = new float[3];
    if(X > LOOKAT_RANGE.MAX_X) X = LOOKAT_RANGE.MAX_X;
    if(X < LOOKAT_RANGE.MIN_X) X = LOOKAT_RANGE.MIN_X;

    if(Y > LOOKAT_RANGE.MAX_Y) Y = LOOKAT_RANGE.MAX_Y;
    if(Y < LOOKAT_RANGE.MIN_Y) Y = LOOKAT_RANGE.MIN_Y;

    if(Z > LOOKAT_RANGE.MAX_Z) Z = LOOKAT_RANGE.MAX_Z;
    if(Z < LOOKAT_RANGE.MIN_Z) Z = LOOKAT_RANGE.MIN_Z;

    lookatTarget[0] = X;
    lookatTarget[1] = Y;
    lookatTarget[2] = Z;

    lookat_set = false;
  }

  void set_ik( String s, float X, float Y, float Z, float T, float N, float v, float a )
  {
    ikString = s;
    ikTarget = new float[5];
    ikAcc = new float[2];
    System.out.println( s + "/" + X + "/" + Y + "/" + Z );
    /*
    if(X > IK_RANGE.MAX_X*10000) X = IK_RANGE.MAX_X*10000;
    if(X < IK_RANGE.MIN_X*10000) X = IK_RANGE.MIN_X*10000;

    if(Y > IK_RANGE.MAX_Y*10000) Y = IK_RANGE.MAX_Y*10000;
    if(Y < IK_RANGE.MIN_Y*10000) Y = IK_RANGE.MIN_Y*10000;

    if(Z > IK_RANGE.MAX_Z*10000) Z = IK_RANGE.MAX_Z*10000;
    if(Z < IK_RANGE.MIN_Z*10000) Z = IK_RANGE.MIN_Z*10000;

    if(T > IK_RANGE.MAX_THETA*10000) T = IK_RANGE.MAX_THETA*10000;
    if(T < IK_RANGE.MIN_THETA*10000) T = IK_RANGE.MIN_THETA*10000;

    if(N > IK_RANGE.MAX_NECK*10000) N = IK_RANGE.MAX_NECK*10000;
    if(N < IK_RANGE.MIN_NECK*10000) N = IK_RANGE.MIN_NECK*10000;
    */
    ikTarget[0] = X;
    ikTarget[1] = Y;
    ikTarget[2] = Z;
    ikTarget[3] = T;
    ikTarget[4] = N;

    ik_set = false;

    if( v > 0.01 && a > 0.01 )
    {
      ikAcc[0] = v;
      ikAcc[1] = a;
      ik_param_set = false;
    }


  }

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

    viseme_set = false;
  }


  void set_blink( String goal )
  {
    if(goal.equalsIgnoreCase("off"))
    {
      blinkTarget = EYES.BLINK_HOLD_STOP;
    }
    else
    {

      if(goal.equalsIgnoreCase("ONCE"))
        blinkTarget=(EYES.BLINK);
      else if(goal.equalsIgnoreCase("START"))
        blinkTarget=(EYES.BLINK_HOLD_START);
      else if(goal.equalsIgnoreCase("STOP"))
        blinkTarget=(EYES.BLINK_HOLD_STOP);
    }

    blink_set = false;
  }

  void set_expression( String constant, String type )
  {
    System.out.println( "type: " + type + " constant: " + constant );
    if(type.equalsIgnoreCase("expression")) 
    { 
      if(constant.equalsIgnoreCase("angry"))
        expressionTarget = (EXPRESSION.EXPRESSION_ANGRY); 
      else if(constant.equalsIgnoreCase("disgusted")) 
        expressionTarget = (EXPRESSION.EXPRESSION_DISGUSTED); 
      //else if(constant.equalsIgnoreCase("frustrated")) 
      //  expressionTarget = (EXPRESSION.EXPRESSION_FRUSTRATED); 
      else if(constant.equalsIgnoreCase("mischievous")) 
        expressionTarget = (EXPRESSION.EXPRESSION_MISCHIEVOUS); 
      //else if(constant.equalsIgnoreCase("shy")) 
      //  expressionTarget = (EXPRESSION.EXPRESSION_SHY); 
      //else if(constant.equalsIgnoreCase("bored")||constant.equalsIgnoreCase("bored_unimpressed")) 
      //  expressionTarget = (EXPRESSION.EXPRESSION_BORED_UNIMPRESSED); 
      //else if(constant.equalsIgnoreCase("ecstatic")) 
      //  expressionTarget = (EXPRESSION.EXPRESSION_ECSTATIC); 
      //else if(constant.equalsIgnoreCase("happy")) 
      //  expressionTarget = (EXPRESSION.EXPRESSION_HAPPY); 
      else if(constant.equalsIgnoreCase("puppy")) 
        expressionTarget = (EXPRESSION.EXPRESSION_PUPPY); 
      else if(constant.equalsIgnoreCase("surprised")) 
        expressionTarget = (EXPRESSION.EXPRESSION_SURPRISED); 
      else if(constant.equalsIgnoreCase("confused")) 
        expressionTarget = (EXPRESSION.EXPRESSION_CONFUSED); 
      else if(constant.equalsIgnoreCase("frightened")) 
        expressionTarget = (EXPRESSION.EXPRESSION_FRIGHTENED); 
      else if(constant.equalsIgnoreCase("lovestruck")) 
        expressionTarget = (EXPRESSION.EXPRESSION_LOVESTRUCK);
      else if(constant.equalsIgnoreCase("tasting")) 
        expressionTarget = (EXPRESSION.EXPRESSION_TASTING);
      else if(constant.equalsIgnoreCase("sipping")) 
        expressionTarget = (EXPRESSION.EXPRESSION_SIPPING); 
      else if(constant.equalsIgnoreCase("sad")) 
        expressionTarget = (EXPRESSION.EXPRESSION_SAD);
      expressionString = constant;
      expression_set = false;
    }
    else if( type.equalsIgnoreCase("motion") )
    {
      if(constant.equalsIgnoreCase("afraid"))
        motionTarget = (MOTION.MOTION_AFRAID);
      else if(constant.equalsIgnoreCase("blech"))
        motionTarget = (MOTION.MOTION_BLECH);
      else if(constant.equalsIgnoreCase("farted"))
        motionTarget = (MOTION.MOTION_FARTED);
      else if(constant.equalsIgnoreCase("idunno"))
        motionTarget = (MOTION.MOTION_IDUNNO);
      else if(constant.equalsIgnoreCase("interest"))
        motionTarget = (MOTION.MOTION_INTEREST);
      else if(constant.equalsIgnoreCase("mmhmmm"))
        motionTarget = (MOTION.MOTION_MMHMMM);
      else if(constant.equalsIgnoreCase("no"))
        motionTarget = (MOTION.MOTION_NO);
      //else if(constant.equalsIgnoreCase("shy"))
      //  motionTarget = (MOTION.MOTION_SHY);
      else if(constant.equalsIgnoreCase("think"))
        motionTarget = (MOTION.MOTION_THINK);
      //else if(constant.equalsIgnoreCase("woah"))
      //  motionTarget = (MOTION.MOTION_WOAH);
      else if(constant.equalsIgnoreCase("yes"))
        motionTarget = (MOTION.MOTION_YES);
      else if(constant.equalsIgnoreCase("anticipation"))
        motionTarget = (MOTION.MOTION_ANTICIPATION);
      //else if(constant.equalsIgnoreCase("cheer"))
      //  motionTarget = (MOTION.MOTION_CHEER);
      //else if(constant.equalsIgnoreCase("heh"))
      //  motionTarget = (MOTION.MOTION_HEH);
      else if(constant.equalsIgnoreCase("ilikeit") || constant.equalsIgnoreCase("i_like_it"))
        motionTarget = (MOTION.MOTION_I_LIKE_IT);
      else if(constant.equalsIgnoreCase("laugh") ||constant.equalsIgnoreCase("laugh1"))
        motionTarget = (MOTION.MOTION_LAUGH1);
      else if(constant.equalsIgnoreCase("mph"))
        motionTarget = (MOTION.MOTION_MPH);
      else if(constant.equalsIgnoreCase("question"))
        motionTarget = (MOTION.MOTION_QUESTION);
      //else if(constant.equalsIgnoreCase("sneeze"))
      //  motionTarget = (MOTION.MOTION_SNEEZE);
      else if(constant.equalsIgnoreCase("wakeup"))
        motionTarget = (MOTION.MOTION_WAKEUP);
      else if(constant.equalsIgnoreCase("yay"))
        motionTarget = (MOTION.MOTION_YAY);
      else if(constant.equalsIgnoreCase("yummm"))
        motionTarget = (MOTION.MOTION_YUMMM);
      else if(constant.equalsIgnoreCase("bite"))
        motionTarget = (MOTION.MOTION_BITE);
      //else if(constant.equalsIgnoreCase("crazy_laugh"))
      //  motionTarget = (MOTION.MOTION_CRAZY_LAUGH);
      else if(constant.equalsIgnoreCase("hungry"))
        motionTarget = (MOTION.MOTION_HUNGRY);
      else if(constant.equalsIgnoreCase("iwantit") ||constant.equalsIgnoreCase("i_want_it"))
        motionTarget = (MOTION.MOTION_I_WANT_IT);
      else if(constant.equalsIgnoreCase("meh"))
        motionTarget = (MOTION.MOTION_MEH);
      //else if(constant.equalsIgnoreCase("nah_nah") || constant.equalsIgnoreCase("nahnah"))
      //  motionTarget = (MOTION.MOTION_NAH_NAH);
      //else if(constant.equalsIgnoreCase("sad"))
      //  motionTarget = (MOTION.MOTION_SAD);
      else if(constant.equalsIgnoreCase("surprise"))
        motionTarget = (MOTION.MOTION_SURPRISE);
      //else if(constant.equalsIgnoreCase("weee"))
      //  motionTarget = (MOTION.MOTION_WEEE);
      else if(constant.equalsIgnoreCase("yawn"))
        motionTarget = (MOTION.MOTION_YAWN);
      motionString = constant;
      motion_set = false;
    }

  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("dragonbot_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    

    int count =1 ;
    final Log log = connectedNode.getLog();

    final Publisher<dragon_msgs.DragonbotStatus> publisher = 
      connectedNode.newPublisher("dragon_status", dragon_msgs.DragonbotStatus._TYPE);

    /******** Viseme Server ********/

    Subscriber<dragon_msgs.VisemeGoal> viseme_subscriber = connectedNode.newSubscriber("dragonbot_viseme", dragon_msgs.VisemeGoal._TYPE);
    viseme_subscriber.addMessageListener(new MessageListener<dragon_msgs.VisemeGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.VisemeGoal goal) {
        System.out.println( "setting viseme: " + goal.getConstant() );
        set_viseme(goal.getConstant());
      }
    });

    /******** Blink Server ********/

    Subscriber<dragon_msgs.BlinkGoal> blink_subscriber = connectedNode.newSubscriber("dragonbot_blink", dragon_msgs.BlinkGoal._TYPE);
    blink_subscriber.addMessageListener(new MessageListener<dragon_msgs.BlinkGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.BlinkGoal goal) {
        System.out.println( "setting blink: " + goal.getConstant() );
        set_blink(goal.getConstant());
      }
    });

    /******** Expression/Motion Server ********/

    Subscriber<dragon_msgs.ExpressionMotionGoal> expression_subscriber = connectedNode.newSubscriber("dragonbot_expression", dragon_msgs.ExpressionMotionGoal._TYPE);
    expression_subscriber.addMessageListener(new MessageListener<dragon_msgs.ExpressionMotionGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.ExpressionMotionGoal goal) {
        set_expression(goal.getConstant(), goal.getType());
      }
    });
    /******** IK Server ********/
    Subscriber<dragon_msgs.IKGoal> ik_subscriber = connectedNode.newSubscriber("dragonbot_ik", dragon_msgs.IKGoal._TYPE);
    ik_subscriber.addMessageListener(new MessageListener<dragon_msgs.IKGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.IKGoal goal) {
        set_ik(goal.getState(), (float) goal.getX(), (float) goal.getY(), (float) goal.getZ(), (float) goal.getTheta(), (float) goal.getNeck(), (float) goal.getVel(), (float) goal.getAcc());
      }
    });

    
   /******** LookAt Server ********/
    Subscriber<dragon_msgs.LookatGoal> lookat_subscriber = connectedNode.newSubscriber("dragonbot_lookat", dragon_msgs.LookatGoal._TYPE);
    lookat_subscriber.addMessageListener(new MessageListener<dragon_msgs.LookatGoal>() {
      @Override
      public void onNewMessage(dragon_msgs.LookatGoal goal) {
        set_lookat(goal.getState(), goal.getX(), goal.getY(), goal.getZ() );
      }
    });

    /******************************/
      // runloop

    connectedNode.executeCancellableLoop( new CancellableLoop() {
      private int sequenceNumber;
      private DragonBotComm comm = null;
      private boolean connected = false;

      @Override
      protected void setup() {
        comm = new DragonBotComm();
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {
        try {
          dragon_msgs.DragonbotStatus status = publisher.newMessage();
          Thread.sleep(33);
          comm.update();
          if(sequenceNumber%100 ==0) comm.sendNetworkDebug(sequenceNumber/100);
          if( !connected && Float.isNaN( comm.getFaceDisplayFPS() ) ) {
            System.out.print("-");
            sequenceNumber++;
            return;
          }
          connected = true;
          System.out.print("+");

          if( visemeTarget != VISEME.IDLE  && !viseme_set ) {
            comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
            comm.sendViseme(visemeTarget);
            viseme_set = true;
          }
          else if ( visemeTarget == VISEME.IDLE && !viseme_set )        
          {
            comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
            viseme_set = true;
          }

          if( blinkTarget != null  && !blink_set ) {
            comm.sendBlink(blinkTarget);
            blink_set = true;
          }
          
          if( !expression_set && expressionTarget != null ) {
            comm.sendExpression(expressionTarget);
            System.out.println( expressionString + "/" + comm.getExpressionCurrent() );
            if( !comm.getExpressionCurrent().equalsIgnoreCase( "IDLE" ) )
              expression_set = true;
          }

          if( !motion_set && motionTarget != null ) {
            comm.sendMotion(motionTarget);
            if( !comm.getMotionCurrent().equalsIgnoreCase( "IDLE" ) )
              motion_set = true;
          }

          if( !lookat_set && lookatString.equalsIgnoreCase("off") )
          {
            comm.sendOnOffControl(LOOKAT_CTRL.TURN_OFF);
            comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
            lookat_set = true;
          }
          else if( !lookat_set && lookatString.equalsIgnoreCase("random") )
          {
            comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_ON);
            lookat_set = true;
          }
          else if( !lookat_set )
          {
            comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
            comm.sendOnOffControl(LOOKAT_CTRL.TURN_ON);
            comm.sendLookat(lookatTarget);
            lookat_set = true;
          }
  
          if( !ik_set && ikString.equalsIgnoreCase("off") )
          {
            comm.sendOnOffControl(IK_CTRL.TURN_OFF);
            ik_set = true;
          }
          else if( !ik_set )
          {
            if( !comm.isActivated(IK_CTRL.QUERY) )
            {
              comm.sendOnOffControl(IK_CTRL.TURN_ON);            
            }
            System.out.println( ikTarget[0] + "/" + ikTarget[1] + "/" + ikTarget[2] );
            comm.setIKfilters( ikAcc[0], ikAcc[1] );
            comm.sendIK(ikTarget);
            float[] ikCurr = comm.getIKCurrent();
            System.out.println( ikCurr[0] + "/" + ikCurr[1] + "/" + ikCurr[2] );

            ik_set = true;
            for( int i = 0; i < 5; i++ )
            {
              if( Double.compare( ikTarget[i], ikCurr[i] ) != 0 ) ik_set = false;
            }
          }

          status.setMotion( comm.getMotionCurrent() );
          status.setExpression( comm.getExpressionCurrent() );

          comm.getLookatTargetCurrent();
          status.setViseme( comm.getVisemeTargetCurrent() );

          status.setBlink( comm.getBlinkCurrent() );
        
          publisher.publish(status);
          comm.getPoseTargetCurrent();


        }
        catch (Exception e) {
          System.out.println( "Exception: " + e.toString() );
          e.printStackTrace();
        }
      }
    });

    /******************************/

  }

}
