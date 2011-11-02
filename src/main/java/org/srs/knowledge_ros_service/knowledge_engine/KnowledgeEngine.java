package org.srs.knowledge_ros_service.knowledge_engine;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;

import com.hp.hpl.jena.query.Query;
import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;

import java.io.*;
import java.util.ArrayList; 

import ros.*;
import ros.communication.*;
import ros.pkg.knowledge_ros_service.srv.AskForActionSequence;  // deprecated
import ros.pkg.knowledge_ros_service.srv.GenerateSequence;
import ros.pkg.knowledge_ros_service.srv.QuerySparQL;
import ros.pkg.knowledge_ros_service.msg.*;
import ros.pkg.knowledge_ros_service.srv.PlanNextAction;
import ros.pkg.knowledge_ros_service.srv.TaskRequest;

import org.srs.knowledge_ros_service.task.*;

import java.util.Properties;

class KnowledgeEngine
{
    public KnowledgeEngine(String nodeName, String ontologyFile)
    {
	ontoDB = new OntologyDB(ontologyFile);
	this.nodeName = nodeName;
	//this.initROS();
    }

    public KnowledgeEngine(Properties conf)
    {
	String ontoDBFile = conf.getProperty("ontologyFile", "/home/ze/ros_workspace/knowledge_ros_service/conf/house.owl");
	ontoDB = new OntologyDB(ontoDBFile);

	this.nodeName = conf.getProperty("nodename", "knowledge_srs_node");
	this.config = conf;

	taskRequestService = conf.getProperty("taskRequestService", "task_request");
	planNextActionService = conf.getProperty("planNextActionService", "plan_next_action");
	generateSequenceService = conf.getProperty("generateSequenceService", "generate_sequence");
	querySparQLService = conf.getProperty("querySparQLService", "query_sparql");
	
	//this.initROS();
    }

    public boolean initROS()
    {
	ros = Ros.getInstance();
	ros.init(nodeName);
	ros.logInfo("INFO: Start RosJava_JNI service");
	//ros.logDebug("DEBUG");
	//ros.logWarn("WARN");
	//ros.logError("ERROR");
	//ros.logFatal("FATAL");
	
	n = ros.createNodeHandle();

	try{
	    initGenerateSequence();
	    initQuerySparQL();
	    initPlanNextAction();
	    initTaskRequest();
	}
	catch(RosException e){
	    System.out.println(e.getMessage());
	    return false;
	}

	ros.spin();
	return true;
    }

    private GenerateSequence.Response handleGenerateSequence(GenerateSequence.Request request)
    {
	GenerateSequence.Response res = new GenerateSequence.Response();
	ArrayList<CUAction> actSeq = new ArrayList<CUAction>();

	CUAction ca = new CUAction(); 
	actSeq.add(ca);
	actSeq.add(ca);
	
	res.actionSequence = actSeq;
	System.out.println(res.actionSequence.size());

	ros.logInfo("INFO: Generate sequence of length: " + actSeq.size());
	return res;
    }
    
    private void initGenerateSequence() throws RosException
    {
	ServiceServer.Callback<GenerateSequence.Request, GenerateSequence.Response> scb = new ServiceServer.Callback<GenerateSequence.Request,GenerateSequence.Response>() {
            public GenerateSequence.Response call(GenerateSequence.Request request) {
		return handleGenerateSequence(request);
            }
	};

	ServiceServer<GenerateSequence.Request,GenerateSequence.Response,GenerateSequence> srv = n.advertiseService(generateSequenceService, new GenerateSequence(), scb);
    }

    private QuerySparQL.Response handleQuerySparQL(QuerySparQL.Request req)
    {
	QuerySparQL.Response re = new QuerySparQL.Response();
	String queryString = req.query;
	System.out.println(queryString);

	re.result = ontoDB.executeQuery(queryString);
	return re;
    }

    private void initQuerySparQL() throws RosException
    {
	ServiceServer.Callback<QuerySparQL.Request, QuerySparQL.Response> scb = new ServiceServer.Callback<QuerySparQL.Request, QuerySparQL.Response>() {
            public QuerySparQL.Response call(QuerySparQL.Request request) {
		return handleQuerySparQL(request);
            }
	};

	ServiceServer<QuerySparQL.Request, QuerySparQL.Response, QuerySparQL> srv = n.advertiseService( querySparQLService , new QuerySparQL(), scb);
    }

    private PlanNextAction.Response handlePlanNextAction( PlanNextAction.Request request)
    {
	PlanNextAction.Response res = new PlanNextAction.Response();

	System.out.println("1 -------------");
	ActionTuple at = null;
	if(request.stateLastAction.length == 3) {
	    if(request.stateLastAction[0] == 0 && request.stateLastAction[1] == 0 && request.stateLastAction[2] == 0) {
		at = currentTask.getNextAction(true);
	    }
	}
	if (at == null) {
	    at = currentTask.getNextAction(false);
	}

	System.out.println("2 -------------");
	
	CUAction ca = new CUAction(); 
	ca = at.getCUAction();

	res.nextAction = ca;
	
	System.out.println("3 -------------");
	
	//ros.logInfo("INFO: Generate sequence of length: ");
	return res;
    }
    
    private void initPlanNextAction() throws RosException
    {
	ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response> scb = new ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response>() {
            public PlanNextAction.Response call(PlanNextAction.Request request) {
		return handlePlanNextAction(request);
            }
	};

	ServiceServer<PlanNextAction.Request, PlanNextAction.Response, PlanNextAction> srv = n.advertiseService(planNextActionService, new PlanNextAction(), scb);
    }

    private TaskRequest.Response handleTaskRequest( TaskRequest.Request request)
    {
	TaskRequest.Response res = new TaskRequest.Response();

	res.result = 0;
	//CUAction ca = new CUAction(); 
		
	//res.nextAction = ca;
	
	//ros.logInfo("INFO: Generate sequence of length: ");
	return res;
    }
    
    private void initTaskRequest() throws RosException
    {
	ServiceServer.Callback<TaskRequest.Request, TaskRequest.Response> scb = new ServiceServer.Callback<TaskRequest.Request, TaskRequest.Response>() {
            public TaskRequest.Response call(TaskRequest.Request request) {
		return handleTaskRequest(request);
            }
	};

	ServiceServer<TaskRequest.Request, TaskRequest.Response, TaskRequest> srv = n.advertiseService(taskRequestService, new TaskRequest(), scb);
    }

    public boolean loadPredefinedTasksForTest()
    {
	try{
	    System.out.println("Create Task Object");
	    currentTask = new Task(Task.TaskType.GET_OBJECT);
	    String taskFile = config.getProperty("taskfile", "task1.seq");
	    System.out.println(taskFile);
	    if(currentTask.loadPredefinedSequence(taskFile))   {
		System.out.println("OK... ");
	    }
	    else  {
		System.out.println("Fail to load... ");
	    }
	}
	catch(Exception e) {
	    // should throw out. only keep for testing here
	    System.out.println(e.getMessage());
	    return false;
	}
	//    public ArrayList<ActionTuple> getActionSequence()
	ArrayList<ActionTuple> acts = currentTask.getActionSequence();

	System.out.println(acts.size());
	return true;
    }

    public static void testTask(Properties conf)
    {
	try{
	    System.out.println("Create Task Object");
	    Task task = new Task(Task.TaskType.GET_OBJECT);
	    String taskFile = conf.getProperty("taskfile", "task1.seq");
	    System.out.println(taskFile);
	    if(task.loadPredefinedSequence(taskFile))   {
		System.out.println("OK... ");
	    }
	    else  {
		System.out.println("Fail to load... ");
	    }
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
    }

    
    
    public static void main(String[] args)
    {
	String configFile = new String();
	System.out.print("There are " + args.length + " input arguments: ");
	if(args.length == 1) {
	    configFile = args[0];
	    System.out.println(configFile);
	}
	else  {
	    configFile = "conf/srsknow.cfg";
	    System.out.println(configFile);
	}

	Properties conf = new Properties();
	KnowledgeEngine knowEng;
	try{
	    InputStream is = new FileInputStream(configFile);
	    conf.load(is);
	
	//////
	//testTask(conf);
	/////


	    knowEng = new KnowledgeEngine(conf);

	    
	}
	catch(IOException e){
	    knowEng = new KnowledgeEngine("knowledge_srs_node", "../conf/house.owl");
	}

	knowEng.loadPredefinedTasksForTest();
	knowEng.initROS();

	
    }

    private Task currentTask;

    private OntologyDB ontoDB;
    private Ros ros;
    private NodeHandle n;
    private String nodeName;

    private Properties config;

    private String taskRequestService = "task_request";
    private String planNextActionService = "plan_next_action";
    private String generateSequenceService = "generate_sequence";
    private String querySparQLService = "query_sparql";

    // 0: normal mode; 1: test mode (no inference, use predefined script instead)  ---- will remove this flag eventually. only kept for testing
    int flag = 1;
}
