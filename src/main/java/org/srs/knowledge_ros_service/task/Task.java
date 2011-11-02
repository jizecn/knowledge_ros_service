package org.srs.knowledge_ros_service.task;

//import 
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.knowledge_ros_service.msg.*;

class Task
{
    public enum TaskType {GET_OBJECT, MOVETO_LOCATION, DETECT_OBJECT, SCAN_AROUND}

    public Task(TaskType type)
    {
	actionSequence = new ArrayList<CUAction>();
	setTaskType(type);
    }

    public void setTaskId(int id)
    {
	this.taskId = id;
    }

    public int getTaskId()
    {
	return taskId;
    }

    public void setTaskType(TaskType type)
    {
	this.taskType = type;
    }

    public ArrayList getActionSequence()
    {
	return actionSequence;
    }

    public void loadPredefinedSequence(String file) throws IOException
    {
	
    }

    public void parseAction(String actionDesc)
    {
	
    }

    private void parseMoveAction(String moveAction)
    {
	
    }

    private TaskType taskType;
    private int taskId;
    private ArrayList<CUAction> actionSequence;
}
