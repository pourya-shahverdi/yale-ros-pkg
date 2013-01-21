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

import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.server.ActionServerCallbacks;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import org.ros.exception.RosException;
import dragon_msgs.DragonAction;
import dragon_msgs.DragonActionFeedback;
import dragon_msgs.DragonActionGoal;
import dragon_msgs.DragonActionResult;
import dragon_msgs.DragonFeedback;
import dragon_msgs.DragonGoal;
import dragon_msgs.DragonResult;

/**
 * The DragonActionSpec class represents the action specification for the
 * Dragon action. It completely hides the Generics approach of the Actionlib
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionSpec
 */
public class DragonActionSpec
    extends
    ActionSpec<DragonAction, DragonActionFeedback, DragonActionGoal, DragonActionResult, DragonFeedback, DragonGoal, DragonResult> {

  /**
   * Constructor to create an action specification for the Dragon action.
   */
  public DragonActionSpec() throws RosException {
    super(DragonAction.class, "dragon_msgs/DragonAction",
        "dragon_msgs/DragonActionFeedback", "dragon_msgs/DragonActionGoal",
        "dragon_msgs/DragonActionResult", "dragon_msgs/DragonFeedback",
        "dragon_msgs/DragonGoal", "dragon_msgs/DragonResult");
  }

  @Override
  public DragonActionClient buildActionClient(String nameSpace) {

    DragonActionClient ac = null;
    try {
      ac = new DragonActionClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return ac;

  }

  @Override
  public DragonSimpleActionClient buildSimpleActionClient(String nameSpace) {

    DragonSimpleActionClient sac = null;
    try {
      return new DragonSimpleActionClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return sac;

  }

  @Override
  public
      DragonActionServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<DragonActionFeedback, DragonActionGoal, DragonActionResult, DragonFeedback, DragonGoal, DragonResult> callbacks) {

    return new DragonActionServer(nameSpace, this, callbacks);

  }

  @Override
  public
      DragonSimpleActionServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<DragonActionFeedback, DragonActionGoal, DragonActionResult, DragonFeedback, DragonGoal, DragonResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new DragonSimpleActionServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
