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

import org.ros.actionlib.client.ActionClient;
import org.ros.exception.RosException;
import dragon_msgs.ExpressionMotionActionFeedback;
import dragon_msgs.ExpressionMotionActionGoal;
import dragon_msgs.ExpressionMotionActionResult;
import dragon_msgs.ExpressionMotionFeedback;
import dragon_msgs.ExpressionMotionGoal;
import dragon_msgs.ExpressionMotionResult;

/**
 * The ExpressionMotionActionClient is a specialized ActionClient that is intended to
 * work with an action server offering services related to the ExpressionMotion action.
 * The ExpressionMotionActionClient completely hides the Generics approach of the
 * ActionClient's implementation.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @see ActionClient
 */
public class ExpressionMotionActionClient
    extends
    ActionClient<ExpressionMotionActionFeedback, ExpressionMotionActionGoal, ExpressionMotionActionResult, ExpressionMotionFeedback, ExpressionMotionGoal, ExpressionMotionResult> {

  public ExpressionMotionActionClient(String nameSpace, ExpressionMotionActionSpec spec) throws RosException {
    super(nameSpace, spec);
  }
}
