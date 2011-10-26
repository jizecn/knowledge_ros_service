import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;
//import com.hp.hpl.jena.graph.query;
import com.hp.hpl.jena.query.Query;

import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
//import com.hp.hpl.jena.vocabulary.*;

import java.io.*;

class TestROSJavaJNI
{
    public TestROSJavaJNI()
    {
    }

    public void testModel()
    {
        // some definitions
        String personURI    = "http://somewhere/JohnSmith";
        String givenName    = "John";
        String familyName   = "Smith";
        String fullName     = givenName + " " + familyName;
        // create an empty model
        Model model = ModelFactory.createDefaultModel();

        // create the resource
        //   and add the properties cascading style
        Resource johnSmith 
          = model.createResource(personURI)
                 .addProperty(VCARD.FN, fullName)
                 .addProperty(VCARD.N, 
                              model.createResource()
                                   .addProperty(VCARD.Given, givenName)
                                   .addProperty(VCARD.Family, familyName));
        
        // list the statements in the graph
        StmtIterator iter = model.listStatements();
        
        // print out the predicate, subject and object of each statement
        while (iter.hasNext()) {
            Statement stmt      = iter.nextStatement();         // get next statement
            Resource  subject   = stmt.getSubject();   // get the subject
            Property  predicate = stmt.getPredicate(); // get the predicate
            RDFNode   object    = stmt.getObject();    // get the object
            
            System.out.print(subject.toString());
            System.out.print(" " + predicate.toString() + " ");
            if (object instanceof Resource) {
                System.out.print(object.toString());
            } else {
                // object is a literal
                System.out.print(" \"" + object.toString() + "\"");
            }
            System.out.println(" .");
        }
    }
    
    public Model loadOWLFile(String file)
    {
	// create an empty model
	Model model = ModelFactory.createDefaultModel();
	
	// use the FileManager to find the input file
	InputStream in = FileManager.get().open( file );
	if (in == null) {
	    throw new IllegalArgumentException(
					       "File: " + file + " not found");
	}
	
	// read the RDF/XML file
	model.read(in, null);
        StmtIterator iter = model.listStatements();
        
        // print out the predicate, subject and object of each statement
        while (iter.hasNext()) {
            Statement stmt      = iter.nextStatement();         // get next statement
            Resource  subject   = stmt.getSubject();   // get the subject
            Property  predicate = stmt.getPredicate(); // get the predicate
            RDFNode   object    = stmt.getObject();    // get the object
	    /*        
    
            System.out.print(subject.toString());
            System.out.print(" " + predicate.toString() + " ");
            if (object instanceof Resource) {
                System.out.print(object.toString());
            } else {
                // object is a literal
                System.out.print(" \"" + object.toString() + "\"");
            }
            System.out.println(" .");
	    */
        }
	return model;
	// write it to standard out
	//model.write(System.out);
    }

    public void testSparQL(Model model)
    {
	String queryString = "PREFIX house: <http://www.semanticweb.org/ontologies/house.owl#> " + 
	    "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> " + 
	    "SELECT ?room " + 
	    "WHERE { " +
	    "?room rdf:type house:Table . " +  
	    "}";
	/*
	    "?room house:containsImmobileObjects ?objects . " +

	String queryString = "SELECT ?objects " + 
	    "WHERE { " + 
	    "<http://www.semanticweb.org/ontologies/house.owl#kitchen1> <http://www.semanticweb.org/ontologies/house.owl#containsimmobileObjects> ?objects .} ";
	*/
	    /*	    "PREFIX foaf: <http://www.semanticweb.org/ontologies/house.owl#> " +
	    "SELECT ?url " +
	    "WHERE {" +
	    "      ?contributor foaf:name \"Jon Foobar\" . " +
	    "      ?contributor foaf:weblog ?url . " +
	    "      }";
	    */
	Query query = QueryFactory.create(queryString);

	QueryExecution qe = QueryExecutionFactory.create(query, model);
	ResultSet results = qe.execSelect();

	ResultSetFormatter.out(System.out, results, query);

	qe.close();
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
	TestROSJavaJNI testRosJava = new TestROSJavaJNI();
	try{
	    Model m = testRosJava.loadOWLFile(args[0]);
	    testRosJava.testSparQL(m);
	}
	catch(IllegalArgumentException e) {
	    System.out.println(e.getMessage());
	}
    }
}