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
import dragon_msgs.IKAction;
import dragon_msgs.IKActionFeedback;
import dragon_msgs.IKActionGoal;
import dragon_msgs.IKActionResult;
import dragon_msgs.IKFeedback;
import dragon_msgs.IKGoal;
import dragon_msgs.IKResult;

/**
 * The IKActionSpec class represents the action specification for the
 * IK action. It completely hides the Generics approach of the Actionlib
 * implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionSpec
 */
public class IKActionSpec
    extends
    ActionSpec<IKAction, IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> {

  /**
   * Constructor to create an action specification for the IK action.
   */
  public IKActionSpec() throws RosException {
    super(IKAction.class, "dragon_msgs/IKAction",
        "dragon_msgs/IKActionFeedback", "dragon_msgs/IKActionGoal",
        "dragon_msgs/IKActionResult", "dragon_msgs/IKFeedback",
        "dragon_msgs/IKGoal", "dragon_msgs/IKResult");
  }

  @Override
  public
      IKActionServer
      buildActionServer(
          String nameSpace,
          ActionServerCallbacks<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> callbacks) {

    return new IKActionServer(nameSpace, this, callbacks);

  }

  @Override
  public
      IKSimpleActionServer
      buildSimpleActionServer(
          String nameSpace,
          SimpleActionServerCallbacks<IKActionFeedback, IKActionGoal, IKActionResult, IKFeedback, IKGoal, IKResult> callbacks,
          boolean useBlockingGoalCallback) {

    return new IKSimpleActionServer(nameSpace, this, callbacks, useBlockingGoalCallback);

  }

}
