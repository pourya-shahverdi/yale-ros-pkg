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
import dragon_msgs.LookatAction;
import dragon_msgs.LookatActionFeedback;
import dragon_msgs.LookatActionGoal;
import dragon_msgs.LookatActionResult;
import dragon_msgs.LookatFeedback;
import dragon_msgs.LookatGoal;
import dragon_msgs.LookatResult;

/**
 * The LookatActionSpec class represents the action specification for the
 * Lookat action. It completely hides the Generics approach of the Actionlib
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionSpec
 */
public class LookatActionSpec
    extends
    ActionSpec<LookatAction, LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> {

  /**
   * Constructor to create an action specification for the Lookat action.
   */
  public LookatActionSpec() throws RosException {
    super(LookatAction.class, "dragon_msgs/LookatAction",
        "dragon_msgs/LookatActionFeedback", "dragon_msgs/LookatActionGoal",
        "dragon_msgs/LookatActionResult", "dragon_msgs/LookatFeedback",
        "dragon_msgs/LookatGoal", "dragon_msgs/LookatResult");
  }

  @Override
  public
      LookatActionServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> callbacks) {

    return new LookatActionServer(nameSpace, this, callbacks);

  }

  @Override
  public
      LookatSimpleActionServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<LookatActionFeedback, LookatActionGoal, LookatActionResult, LookatFeedback, LookatGoal, LookatResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new LookatSimpleActionServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
