package thesisUtilities;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors
import java.io.BufferedWriter;

public class WriteTest {
	private static FileWriter fileHandler;
	private static BufferedWriter printData;
	// old method
	  public static void doStuff(int action, String str) {
	    try {
	    	switch(action){
	    	case 0:
	    		fileHandler = new FileWriter(str);
	    		printData = new BufferedWriter(fileHandler);
	    		break;
	    	case 1:
	  	      	printData.write(str);
	  	      	printData.newLine();
	  	      	break;
	    	case 2:
	    		printData.close();
	    		break;
	    	}
	      
	      //System.out.println("Successfully wrote to the file.");
	    } catch (IOException e) {
	      System.out.println("An error occurred.");
	      e.printStackTrace();
	    }
	  }
	// new method
		public static void init(String fileName){
		    try {
		    	fileHandler = new FileWriter(fileName);
	    		printData = new BufferedWriter(fileHandler);	      
		      //System.out.println("Successfully wrote to the file.");
		    } catch (IOException e) {
		      System.out.println("An error occurred.");
		      e.printStackTrace();
		    }   
		}
		
		public static void saveLine(String str){
		    try {
	  	      	printData.write(str);
	  	      	printData.newLine();
		    } catch (IOException e) {
		      System.out.println("An error occurred.");
		      e.printStackTrace();
		    }   
		}
		
		public static void end(){
		    try {
		    	printData.close();
		    } catch (IOException e) {
		      System.out.println("An error occurred.");
		      e.printStackTrace();
		    }   
		}
	}
