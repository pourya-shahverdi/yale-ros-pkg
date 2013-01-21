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
import dragon_msgs.LookatActionFeedback;
import dragon_msgs.LookatActionFeedback;
import dragon_msgs.LookatActionGoal;
import dragon_msgs.LookatActionResult;
import dragon_msgs.LookatFeedback;
import dragon_msgs.LookatGoal;
import dragon_msgs.LookatResult;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;


/**
 * A {@link SimpleActionServerCallbacks} for the Dragon example.
 */
public class LookatSimpleActionServerCallbacks
    implements
    SimpleActionServerCallbacks<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> {

DragonBotComm comm;

public LookatSimpleActionServerCallbacks(DragonBotComm establishedComm)
{
    comm = establishedComm;
    int count =1 ;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	{
	    //System.out.println("constructor loop");
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		comm.sendNetworkDebug(count/10000);
	
	}

}

  public LookatResult newResultMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(LookatResult._TYPE);
  }

   public LookatFeedback newFeedbackMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(LookatFeedback._TYPE);
  }


  @Override
  public void blockingGoalCallback(LookatGoal goal, SimpleActionServer<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> actionServer) 
{
	int count =1;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	{
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		comm.sendNetworkDebug(count/10000);
	
	}

	    if(goal.getState().equalsIgnoreCase("off"))
		{
			comm.sendOnOffControl(LOOKAT_CTRL.TURN_OFF);
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
		}
	    else if(goal.getState().equalsIgnoreCase("random"))
		{
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_ON);
		}
	    else
		{
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
			comm.sendOnOffControl(LOOKAT_CTRL.TURN_ON);
			float[] currentLookat = new float[3];
			if(goal.getX() > LOOKAT_RANGE.MAX_X) goal.setX( LOOKAT_RANGE.MAX_X );
			if(goal.getX() < LOOKAT_RANGE.MIN_X) goal.setX( LOOKAT_RANGE.MIN_X );

			if(goal.getY() > LOOKAT_RANGE.MAX_Y) goal.setY( LOOKAT_RANGE.MAX_Y );
			if(goal.getY() < LOOKAT_RANGE.MIN_Y) goal.setY( LOOKAT_RANGE.MIN_Y );

			if(goal.getZ() > LOOKAT_RANGE.MAX_Z) goal.setZ( LOOKAT_RANGE.MAX_Z );
			if(goal.getZ() < LOOKAT_RANGE.MIN_Z) goal.setZ( LOOKAT_RANGE.MIN_Z );
			currentLookat[0] = (float)goal.getX();
			currentLookat[1] = (float)goal.getY();
			currentLookat[2] = (float)goal.getZ();
			System.out.println(currentLookat[0]);
			System.out.println(currentLookat[1]);
			System.out.println(currentLookat[2]);
			comm.sendLookat(currentLookat);
			comm.update();
			count++;
			   

		}

}

  @Override
  public
      void
      goalCallback(
          SimpleActionServer<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> actionServer) 
{
System.out.println("GOAL CALLBACK");
LookatGoal goal = null;
try
{
	goal = actionServer.acceptNewGoal();
}
catch (RosException e){}
	int count =1;
    comm.sendNetworkDebug(count);
    while(Float.isNaN(comm.getFaceDisplayFPS()))
	{
	    comm.update();
	    count++;
	    if(count%10000 ==0)
		comm.sendNetworkDebug(count/10000);
	
	}

	    if(goal.getState().equalsIgnoreCase("off"))
		{
			comm.sendOnOffControl(LOOKAT_CTRL.TURN_OFF);
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
		}
	    else if(goal.getState().equalsIgnoreCase("random"))
		{
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_ON);
		}
	    else
		{
			comm.sendOnOffControl(RAND_LOOKAT_CTRL.TURN_OFF);
			comm.sendOnOffControl(LOOKAT_CTRL.TURN_ON);
			float[] currentLookat = new float[3];
			if(goal.getX() > LOOKAT_RANGE.MAX_X) goal.setX(LOOKAT_RANGE.MAX_X);
			if(goal.getX() < LOOKAT_RANGE.MIN_X) goal.setX(LOOKAT_RANGE.MIN_X);

			if(goal.getY() > LOOKAT_RANGE.MAX_Y) goal.setY( LOOKAT_RANGE.MAX_Y );
			if(goal.getY() < LOOKAT_RANGE.MIN_Y) goal.setY( LOOKAT_RANGE.MIN_Y );

			if(goal.getZ() > LOOKAT_RANGE.MAX_Z) goal.setZ( LOOKAT_RANGE.MAX_Z );
			if(goal.getZ() < LOOKAT_RANGE.MIN_Z) goal.setZ( LOOKAT_RANGE.MIN_Z );
			currentLookat[0] = (float)goal.getX();
			currentLookat[1] = (float)goal.getY();
			currentLookat[2] = (float)goal.getZ();
			System.out.println(currentLookat[0]);
			System.out.println(currentLookat[1]);
			System.out.println(currentLookat[2]);
			comm.sendLookat(currentLookat);
			comm.update();
			count++;
			   

		}


}

  @Override
  public
      void
      preemptCallback(
          SimpleActionServer<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> actionServer) {
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
          SimpleActionServer<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> actionServer) {
    LookatFeedback feedback = newFeedbackMessage();
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
