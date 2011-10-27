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
import ros.pkg.knowledge_ros_service.srv.AskForActionSequence;
import ros.pkg.knowledge_ros_service.srv.QuerySparQL;
import ros.pkg.knowledge_ros_service.msg.*;

class KnowledgeEngine
{
    public KnowledgeEngine(String nodeName)
    {
	ontoDB = new OntologyDB("house.owl");
	this.nodeName = nodeName;
	this.initROS();

    }

    private Boolean initROS()
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
	}
	catch(RosException e){
	    System.out.println(e.getMessage());
	    return false;
	}

	ros.spin();
	return true;
    }

    private AskForActionSequence.Response handleGenerateSequence(AskForActionSequence.Request request)
    {
	AskForActionSequence.Response res = new AskForActionSequence.Response();
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
	ServiceServer.Callback<AskForActionSequence.Request, AskForActionSequence.Response> scb = new ServiceServer.Callback<AskForActionSequence.Request,AskForActionSequence.Response>() {
            public AskForActionSequence.Response call(AskForActionSequence.Request request) {
		return handleGenerateSequence(request);
            }
	};

	ServiceServer<AskForActionSequence.Request,AskForActionSequence.Response,AskForActionSequence> srv = n.advertiseService("ask_for_action_sequence", new AskForActionSequence(), scb);
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

	ServiceServer<QuerySparQL.Request, QuerySparQL.Response, QuerySparQL> srv = n.advertiseService("query_sparql", new QuerySparQL(), scb);
    }

    public static void main(String[] args)
    {
	System.out.print("There are " + args.length + " input arguments: ");
	if(args.length == 1) {
	    System.out.println(args[0]);
	}
	else  {
	    System.out.println();
	}
	KnowledgeEngine knowEng = new KnowledgeEngine("knowledge_srs_node");
	
	/*
	try{
	    Model m = testRosJava.loadOWLFile(args[0]);
	    testRosJava.testSparQL(m);
	}
	catch(IllegalArgumentException e) {
	    System.out.println(e.getMessage());
	}
	*/
    }
    private OntologyDB ontoDB;
    private Ros ros;
    private NodeHandle n;
    private String nodeName;
}
