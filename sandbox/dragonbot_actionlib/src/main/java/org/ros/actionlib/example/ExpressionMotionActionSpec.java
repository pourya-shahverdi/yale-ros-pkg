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
import dragon_msgs.ExpressionMotionAction;
import dragon_msgs.ExpressionMotionActionFeedback;
import dragon_msgs.ExpressionMotionActionGoal;
import dragon_msgs.ExpressionMotionActionResult;
import dragon_msgs.ExpressionMotionFeedback;
import dragon_msgs.ExpressionMotionGoal;
import dragon_msgs.ExpressionMotionResult;

/**
 * The ExpressionMotionActionSpec class represents the action specification for the
 * ExpressionMotion action. It completely hides the Generics approach of the Actionlib
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionSpec
 */
public class ExpressionMotionActionSpec
    extends
    ActionSpec<ExpressionMotionAction, ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> {

  /**
   * Constructor to create an action specification for the ExpressionMotion action.
   */
  public ExpressionMotionActionSpec() throws RosException {
    super(ExpressionMotionAction.class, "dragon_msgs/ExpressionMotionAction",
        "dragon_msgs/ExpressionMotionActionFeedback", "dragon_msgs/ExpressionMotionActionGoal",
        "dragon_msgs/ExpressionMotionActionResult", "dragon_msgs/ExpressionMotionFeedback",
        "dragon_msgs/ExpressionMotionGoal", "dragon_msgs/ExpressionMotionResult");
  }

  @Override
  public ExpressionMotionActionClient buildActionClient(String nameSpace) {

    ExpressionMotionActionClient ac = null;
    try {
      ac = new ExpressionMotionActionClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return ac;

  }

  @Override
  public ExpressionMotionSimpleActionClient buildSimpleActionClient(String nameSpace) {

    ExpressionMotionSimpleActionClient sac = null;
    try {
      return new ExpressionMotionSimpleActionClient(nameSpace, this);
    } catch (RosException e) {
      e.printStackTrace();
    }
    return sac;

  }

  @Override
  public
      ExpressionMotionActionServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> callbacks) {

    return new ExpressionMotionActionServer(nameSpace, this, callbacks);

  }

  @Override
  public
      ExpressionMotionSimpleActionServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new ExpressionMotionSimpleActionServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
