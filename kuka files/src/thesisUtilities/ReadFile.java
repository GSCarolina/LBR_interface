package thesisUtilities;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
/**
 *
 * @author Carolina
 */
public class ReadFile {
        private static String strCurrentLine;
        private static BufferedReader objReader = null;
        private static FileReader fileHandler;
        public static List<String> fileLines;
        public static int linesNr; 
        public static boolean error;
        
	public static void init(String fileName){
		error = false;
	    try {
	    	fileHandler = new FileReader(fileName);
                objReader = new BufferedReader(fileHandler);
                // init
                fileLines = new ArrayList<String>();
                fileLines.clear();
                linesNr = 0;
	      //System.out.println("Successfully wrote to the file.");
	    } catch (IOException e) {
	    	error = true;
	      System.out.println("An error occurred.");
	      e.printStackTrace();
	    }   
	}
		
	public static void readContent(){
	    try {
	      	while ((strCurrentLine = objReader.readLine()) != null) {
                        fileLines.add(strCurrentLine);
                        linesNr++;
                        //System.out.println(strCurrentLine);
                    }
	      	error = false;
	    } catch (IOException e) {
	    	error = true;
            System.out.println("An error occurred.");
            e.printStackTrace();
            }   
	}
		
	public static void end(){
	    try {
	    	objReader.close();
	    	error = false;
	    } catch (IOException e) {
	      error = true;
	      System.out.println("An error occurred.");
	      e.printStackTrace();
	    }   
	}
        
        public String[] splitData(String delimiter, int index){
            int i = 0;
            if(index>=0 && index < linesNr){
                i = index;
            }
            String line = fileLines.get(i);
            String[] result = line.split(delimiter);
               
            return result;
        }
} // class
