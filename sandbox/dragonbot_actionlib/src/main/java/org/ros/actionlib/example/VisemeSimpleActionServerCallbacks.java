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
import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionGoal;
import dragon_msgs.VisemeActionResult;
import dragon_msgs.VisemeFeedback;
import dragon_msgs.VisemeGoal;
import dragon_msgs.VisemeResult;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

/**
 * A {@link SimpleActionServerCallbacks} for the Dragon example.
 */
public class VisemeSimpleActionServerCallbacks
    implements
    SimpleActionServerCallbacks<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> {

  DragonBotComm comm;
  boolean preempted = false;

  public VisemeSimpleActionServerCallbacks(DragonBotComm establishedComm)
  {
    comm = establishedComm;
    int count =1 ;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	  {
	    //System.out.println("constructor loop");
	    comm.update();
      try {
        Thread.sleep(33);
      }
      catch(Exception e) {}
	    count++;
	    if(count%10000 ==0)
		    comm.sendNetworkDebug(count/10000);
  	
	  }
  }

  public VisemeResult newResultMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(VisemeResult._TYPE);
  }

   public VisemeFeedback newFeedbackMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(VisemeFeedback._TYPE);
  }

  boolean checkUpdate( SimpleActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> actionServer )
  {
    comm.update();
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
  public void blockingGoalCallback(VisemeGoal goal, SimpleActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> actionServer) 
  {
	/* Viseme Ids
	 * IDLE = 36
	 * AA_AH = 0
	 * AO_AW = 3
	 * CH_SH_ZH = 7
	 * EH_AE_AY = 1
	 * EY = 12
	 * L = 20
	 * M_B_P = 6
	 * N_NG_D_Z = 8
	 * R_ER = 11
	*/
    System.out.println( "VISEME BLOCKING CALLBACK" );
    int count =1 ;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	  {
	    comm.update();
	    count++;
	    if(count%10000 ==0)
	  	comm.sendNetworkDebug(count/10000);
	  }

    if(goal.getConstant().equalsIgnoreCase("off"))
	  {
  		comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
	  }
    else
	  {
		  comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
  		while(checkUpdate(actionServer))
		  {
			  //This makes everything run smoothly... but it seems like it should allow preempting and doesn't
  			if(actionServer.isNewGoalAvailable())
        {
				  try
  				{
	  				goal = actionServer.acceptNewGoal();
		  		}
			  	catch (RosException e){}
			  }
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
        try {
          Thread.sleep(33);
        }
        catch(Exception e) {}
	    }
	  }
  }

  @Override
  public void goalCallback(
          SimpleActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> actionServer) 
  {
    VisemeGoal goal = null;
    try
    {
    	goal = actionServer.acceptNewGoal();
    }
    catch (RosException e){}
	/* Viseme Ids
	 * IDLE = 36
	 * AA_AH = 0
	 * AO_AW = 3
	 * CH_SH_ZH = 7
	 * EH_AE_AY = 1
	 * EY = 12
	 * L = 20
	 * M_B_P = 6
	 * N_NG_D_Z = 8
	 * R_ER = 11
	*/
    int count =1;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	  {
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		    comm.sendNetworkDebug(count/10000);
	  }

    if(goal.getConstant().equalsIgnoreCase("off"))
  	{
	  	comm.sendOnOffControl(VISEME_CTRL.TURN_OFF);
	  }
    else
  	{
	  	comm.sendOnOffControl(VISEME_CTRL.TURN_ON);
		  while(checkUpdate(actionServer))
		  {
			  //This makes everything run smoothly... but it seems like it should allow preempting and doesn't
  			if(actionServer.isNewGoalAvailable())
        {
				  try
  				{
	  				goal = actionServer.acceptNewGoal();
		  		}
			  	catch (RosException e){}
			  }
  			if(count==1)
	      {
          System.out.println( "sending viseme" );
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
  				count = 10000;
			  }
			  comm.update();
			  count--;
        try {
          Thread.sleep(33);
        }
        catch(Exception e) {}
		  }
	  }
  }

  @Override
  public void preemptCallback(
          SimpleActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> actionServer) {
    System.out.println("PREEMPT CALLBACK");
    preempted = true;
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
  private void publishFeedback(
          String status,
          SimpleActionServer<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> actionServer) {
    VisemeFeedback feedback = newFeedbackMessage();
    feedback.setStatus(status);
    actionServer.publishFeedback(feedback);
    //for (int i = 0; i < feedback.sequence.length; i++) {
      //if (feedback.sequence[i] == 0 && i != 0) {
       // break;
      //}
      //System.out.print(" " + feedback.sequence[i]);
    //}
  }

}
