/*
 * Copyright (C) 2011 Alexander Perzylo, Technische Universität München
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.actionlib.example;
import org.ros.exception.RosException;
import org.ros.actionlib.server.SimpleActionServer;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import dragon_msgs.ExpressionMotionActionFeedback;
import dragon_msgs.ExpressionMotionActionFeedback;
import dragon_msgs.ExpressionMotionActionGoal;
import dragon_msgs.ExpressionMotionActionResult;
import dragon_msgs.ExpressionMotionFeedback;
import dragon_msgs.ExpressionMotionGoal;
import dragon_msgs.ExpressionMotionResult;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

/**
 * A {@link SimpleActionServerCallbacks} for the Dragon example.
 */
public class ExpressionMotionSimpleActionServerCallbacks
    implements
    SimpleActionServerCallbacks<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> {

DragonBotComm comm;

  boolean preempted = false;

  public ExpressionMotionSimpleActionServerCallbacks(DragonBotComm establishedComm)
  {
    
    comm = establishedComm;
    int count =1 ;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
  	{
	    //System.out.println("constructor loop");
	    comm.update();
      try{
        Thread.sleep(33);
      }
      catch (Exception e) {}
	    count++;
  	  if(count%1000 ==0)
        //break;
	    	comm.sendNetworkDebug(count/10000);
  	}
  }

  public ExpressionMotionResult newResultMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(ExpressionMotionResult._TYPE);
  }

  public ExpressionMotionFeedback newFeedbackMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(ExpressionMotionFeedback._TYPE);
  }

  boolean checkUpdate( SimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> actionServer )
  {
    comm.update();
    System.out.println( comm.getExpressionCurrent() );
    if( preempted )
    {
      System.out.println( "preempt requested" );
      actionServer.setPreempted();
      preempted = false;
      return false;
    }
    return true;
  }


 @Override
  public void blockingGoalCallback(ExpressionMotionGoal goal, SimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> actionServer) 
  {
    System.out.println("BLOCKING GOAL CALLBACK");

    int count =1;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	  {
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		  comm.sendNetworkDebug(count/10000);
      if( !checkUpdate(actionServer) )
      {
        return;
      }
	  }

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
	    publishFeedback("Expression sent - waiting for execution", actionServer);
	    comm.update();

	    while(comm.getExpressionCurrent().equals("IDLE"))
		  {
        if( !checkUpdate(actionServer) )
        {
          return;
        }
		  }

	    String cur = comm.getExpressionCurrent();
	    publishFeedback("Expression in progress", actionServer);
	 
	    while(comm.getExpressionCurrent().equals(cur))
		  {
        if( !checkUpdate(actionServer) )
        {
          return;
        } 
		  }

	    publishFeedback("Expression Completed", actionServer);
	    ExpressionMotionResult result = newResultMessage();
      result.setResult("The expression completed successfully");
    	actionServer.setSucceeded(result, "");
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
	    else if(goal.getConstant().equalsIgnoreCase("yawm"))
	    	comm.sendMotion(MOTION.MOTION_YAWN);

 	    while(comm.getMotionCurrent().equals("IDLE"))
		  {
        if( !checkUpdate(actionServer) )
        {
          return;
        } 
		  }
	    String cur = comm.getMotionCurrent();
	    publishFeedback("Motion in progress", actionServer);
	 
	    while(comm.getMotionCurrent().equals(cur))
		  {
        if( !checkUpdate(actionServer) )
        {
          return;
        } 
		  }
	    System.out.println("Motion Completed");
	    publishFeedback("Motion Completed", actionServer);
	    ExpressionMotionResult result = newResultMessage();
    	result.setResult("The motion completed successfully");
    	actionServer.setSucceeded(result, "");
	  }
  }


  @Override
  public
      void
      goalCallback(
          SimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> actionServer) {
        ExpressionMotionGoal goal = null;
        try
        {
        	goal = actionServer.acceptNewGoal();
        }
        catch (RosException e){
          System.err.println("Problem with acceptNewGoal: " + e.toString() );  
        }
        System.out.println("GOAL CALLBACK");
        int count =1 ;
        for( int i = 0; i < 100; i++ )
        {
          try{
            Thread.sleep(100);
          } catch (Exception e) {}
          System.out.print("."); 
          if( actionServer.isPreemptRequested() )
          {
            actionServer.setPreempted();
            return;
          }
          
        }
	      publishFeedback("Motion Completed", actionServer);
	      ExpressionMotionResult result = newResultMessage();
    	  result.setResult("The motion completed successfully");
    	  actionServer.setSucceeded(result, "");
      /*
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	{
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		comm.sendNetworkDebug(count/10000);
	    System.out.println("can't get feedback");
	
	}
    System.out.println( "type: " + goal.getType() + "|constant: " + goal.getConstant() );
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
	    publishFeedback("Expression sent - waiting for execution", actionServer);
	    comm.update();

	    while(comm.getExpressionCurrent().equals("IDLE"))
		{

		    comm.update();
		}
	    String cur = comm.getExpressionCurrent();
	    publishFeedback("Expression in progress", actionServer);
	 
	    while(comm.getExpressionCurrent().equals(cur))
		{
		    comm.update();
		}
	   publishFeedback("Expression Completed", actionServer);
	   ExpressionMotionResult result = newResultMessage();
    	   result.setResult("The expression completed successfully");
    	   actionServer.setSucceeded(result, "");
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
	    else if(goal.getConstant().equalsIgnoreCase("yawm"))
	    	comm.sendMotion(MOTION.MOTION_YAWN);

 	    while(comm.getMotionCurrent().equals("IDLE"))
		{
		    comm.update();
		}
	    String cur = comm.getMotionCurrent();
	    publishFeedback("Motion in progress", actionServer);
	 
	    while(comm.getMotionCurrent().equals(cur))
		{
		    comm.update();
		}
	    System.out.println("Motion Completed");
	    publishFeedback("Motion Completed", actionServer);
	    ExpressionMotionResult result = newResultMessage();
    	    result.setResult("The motion completed successfully");
    	    actionServer.setSucceeded(result, "");
	}
*/
  }

  @Override
  public
      void
      preemptCallback(
          SimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> actionServer) {
        //actionServer.setPreempted();
        preempted = true;
        System.out.println("PREEMPT CALLBACK");
  }

  private void snore() {

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
    }

  }

  /**
   * Send feedback to the client on how far along the computation is.
   * 
   * @param seq
   *          The sequence step the computation is on.
   * @param actionServer
   *          The action server publishing information.
   */
  private
      void
      publishFeedback(
          String status,
          SimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> actionServer) {
        ExpressionMotionFeedback feedback = newFeedbackMessage();
        feedback.setStatus(status);
        actionServer.publishFeedback(feedback);
        System.out.println( "published feedback" );
      }

}
