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
import org.ros.actionlib.server.DefaultSimpleActionServer;
import org.ros.actionlib.server.SimpleActionServerCallbacks;
import dragon_msgs.ExpressionMotionActionFeedback;
import dragon_msgs.ExpressionMotionActionGoal;
import dragon_msgs.ExpressionMotionActionResult;
import dragon_msgs.ExpressionMotionFeedback;
import dragon_msgs.ExpressionMotionGoal;
import dragon_msgs.ExpressionMotionResult;

/**
 * The ExpressionMotionSimpleActionServer is a specialized DefaultSimpleActionServer
 * that offers services related to the ExpressionMotion action. The
 * ExpressionMotionSimpleActionServer completely hides the Generics approach of the
 * DefaultSimpleActionServer's implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see DefaultSimpleActionServer
 */
public class ExpressionMotionSimpleActionServer
    extends
    DefaultSimpleActionServer<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> {

  public ExpressionMotionSimpleActionServer(
      String nameSpace,
      ActionSpec<?, ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> spec,
      SimpleActionServerCallbacks<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> callbacks,
      boolean useBlockingGoalCallback) {
    super(nameSpace, spec, callbacks, useBlockingGoalCallback);
  }

}
