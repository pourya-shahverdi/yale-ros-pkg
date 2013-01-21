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
import dragon_msgs.VisemeAction;
import dragon_msgs.VisemeActionFeedback;
import dragon_msgs.VisemeActionGoal;
import dragon_msgs.VisemeActionResult;
import dragon_msgs.VisemeFeedback;
import dragon_msgs.VisemeGoal;
import dragon_msgs.VisemeResult;

/**
 * The VisemeActionSpec class represents the action specification for the
 * Viseme action. It completely hides the Generics approach of the Actionlib
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionSpec
 */
public class VisemeActionSpec
    extends
    ActionSpec<VisemeAction, VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> {

  /**
   * Constructor to create an action specification for the Viseme action.
   */
  public VisemeActionSpec() throws RosException {
    super(VisemeAction.class, "dragon_msgs/VisemeAction",
        "dragon_msgs/VisemeActionFeedback", "dragon_msgs/VisemeActionGoal",
        "dragon_msgs/VisemeActionResult", "dragon_msgs/VisemeFeedback",
        "dragon_msgs/VisemeGoal", "dragon_msgs/VisemeResult");
  }

  @Override
  public
      VisemeActionServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> callbacks) {

    return new VisemeActionServer(nameSpace, this, callbacks);

  }

  @Override
  public
      VisemeSimpleActionServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<VisemeActionFeedback, VisemeActionGoal, VisemeActionResult, VisemeFeedback, VisemeGoal, VisemeResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new VisemeSimpleActionServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
