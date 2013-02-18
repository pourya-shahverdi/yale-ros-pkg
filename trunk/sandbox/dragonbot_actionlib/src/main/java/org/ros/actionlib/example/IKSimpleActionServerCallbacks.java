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
import dragon_msgs.IKActionFeedback;
import dragon_msgs.IKActionFeedback;
import dragon_msgs.IKActionGoal;
import dragon_msgs.IKActionResult;
import dragon_msgs.IKFeedback;
import dragon_msgs.IKGoal;
import dragon_msgs.IKResult;
import prg.content.dragonbot.teleopstandalone.DragonBotComm;
import prg.content.dragonbot.teleopstandalone.Constants.*;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

/**
 * A {@link SimpleActionServerCallbacks} for the Dragon example.
 */
public class IKSimpleActionServerCallbacks
implements
SimpleActionServerCallbacks<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> {

  DragonBotComm comm;
  boolean preempted = false;

  public IKSimpleActionServerCallbacks(DragonBotComm establishedComm)
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
      try {
        Thread.sleep(33);
      }
      catch (Exception E) {}
    }

  }

  public IKResult newResultMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(IKResult._TYPE);
  }

  public IKFeedback newFeedbackMessage()
  {
    NodeConfiguration nc = NodeConfiguration.newPrivate();
    MessageFactory mf = nc.getTopicMessageFactory();
    return mf.newFromType(IKFeedback._TYPE);
  }


  boolean checkUpdate( SimpleActionServer<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> actionServer )
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
    public void blockingGoalCallback(IKGoal goal, SimpleActionServer<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> actionServer) 
    {
      System.out.println( "BLOCKING IK CALLBACK" );
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
        comm.sendOnOffControl(IK_CTRL.TURN_OFF);
      }
      else
      {
        comm.sendOnOffControl(IK_CTRL.TURN_ON);
        float[] currentIK = comm.getIKCurrent();


        Float[] currentFilters = comm.getIKfilters();
        System.out.println( "vel: " + currentFilters[0] + " acc: " + currentFilters[1] );
        currentIK[0] = (float)goal.getX();
        currentIK[1] = (float)goal.getY();
        currentIK[2] = (float)goal.getZ();
        currentIK[3] = (float)goal.getTheta();
        currentIK[4] = (float)goal.getNeck();
        comm.sendOnOffControl(IK_CTRL.TURN_ON);
        count = 0;
        while(checkUpdate(actionServer))
        {
          float[] newIK = comm.getIKCurrent();
          System.out.println( "x: " + newIK[0] + " y: " + newIK[1] + " z: " + newIK[2] + " theta: " + newIK[3] + " neck: " + newIK[4] );
          System.out.println( "x: " + currentIK[0] + " y: " + currentIK[1] + " z: " + currentIK[2] + " theta: " + currentIK[3] + " neck: " + currentIK[4] );
          if( count % 10000 == 0 )
          {
            //comm.setIKfilters((float)goal.getVel(), (float)goal.getAcc());
            comm.sendIK(currentIK);


          }
          comm.update();
          try {
            Thread.sleep(33);
          } catch (Exception e ) {}
          count++;
        }
      }
    }

  @Override
    public void goalCallback(
        SimpleActionServer<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> actionServer) 
    {
      System.out.println("GOAL CALLBACK");
      IKGoal goal = null;
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
        comm.sendOnOffControl(IK_CTRL.TURN_OFF);
      }

      else
      {
        comm.setIKfilters((float)goal.getVel(), (float)goal.getAcc());	
        comm.sendOnOffControl(IK_CTRL.TURN_ON);
        float[] currentIK = new float[5];
        if(goal.getX() > IK_RANGE.MAX_X*10000) goal.setX( IK_RANGE.MAX_X*10000);
        if(goal.getX() < IK_RANGE.MIN_X*10000) goal.setX( IK_RANGE.MIN_X*10000);

        if(goal.getY() > IK_RANGE.MAX_Y*10000) goal.setY( IK_RANGE.MAX_Y*10000);
        if(goal.getY() < IK_RANGE.MIN_Y*10000) goal.setY( IK_RANGE.MIN_Y*10000);

        if(goal.getZ() > IK_RANGE.MAX_Z*10000) goal.setZ( IK_RANGE.MAX_Z*10000);
        if(goal.getZ() < IK_RANGE.MIN_Z*10000) goal.setZ( IK_RANGE.MIN_Z*10000);

        if(goal.getTheta() > IK_RANGE.MAX_THETA*10000) goal.setTheta( IK_RANGE.MAX_THETA*10000);
        if(goal.getTheta() < IK_RANGE.MIN_THETA*10000) goal.setTheta( IK_RANGE.MIN_THETA*10000);

        if(goal.getNeck() > IK_RANGE.MAX_NECK*10000) goal.setNeck( IK_RANGE.MAX_NECK*10000);
        if(goal.getNeck() < IK_RANGE.MIN_NECK*10000) goal.setNeck( IK_RANGE.MIN_NECK*10000);
        currentIK[0] = (float)goal.getX();
        currentIK[1] = (float)goal.getY();
        currentIK[2] = (float)goal.getZ();
        currentIK[3] = (float)goal.getTheta();
        currentIK[4] = (float)goal.getNeck();
        float[] actualIK = comm.getIKCurrent();
        while(!(currentIK[0] >= actualIK[0]-.001 && currentIK[0] <= actualIK[0]+.001) ||!(currentIK[1] >= actualIK[1]-.001 && currentIK[1] <= actualIK[1]+.001) ||!(currentIK[2] >= actualIK[2]-.001 && currentIK[2] <= actualIK[2]+.001)  ||!(currentIK[3] >= actualIK[3]-.001 && currentIK[3] <= actualIK[3]+.001)  ||!(currentIK[4] >= actualIK[4]-.001 && currentIK[4] <= actualIK[4]+.001) )
        {
          comm.sendIK(currentIK);
          comm.update();
          actualIK = comm.getIKCurrent();
          if( !checkUpdate(actionServer) ) break;
        }
      }
    }

  @Override
    public
    void
    preemptCallback(
        SimpleActionServer<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> actionServer) {
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
  private
    void
    publishFeedback(
        String status,
        SimpleActionServer<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> actionServer) {
      IKFeedback feedback = newFeedbackMessage();
      feedback.setStatus( status );
      actionServer.publishFeedback(feedback);
      //for (int i = 0; i < feedback.sequence.length; i++) {
      //if (feedback.sequence[i] == 0 && i != 0) {
      // break;
      //}
      //System.out.print(" " + feedback.sequence[i]);
      //}
    }

}
