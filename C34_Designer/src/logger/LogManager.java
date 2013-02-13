package logger;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Scanner;

public final class LogManager {
	
	private static String _outputFileName;
	private static PrintStream _outputStream;
	
	private static PrintStream createPrintStreamToFile(String fileName) {
		 try {
			return new PrintStream(new BufferedOutputStream(new FileOutputStream(fileName)));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return null;
		}
	}

	public static String getOutputFileName() {
		return _outputFileName;
	}
	
	public static void redirectStandardAndErrorOutput(String outputFileName) {
		_outputFileName = outputFileName;
		_outputStream = createPrintStreamToFile(outputFileName);
		
		System.setOut(_outputStream);
		System.setErr(_outputStream);
	}
	
	public static String getOutputContent() {
		if (_outputFileName == null) 
			return "Output filename not set";
		
		if (_outputStream == null)
			return "Output stream not set";
		
		_outputStream.flush();
			
		try {
			Scanner scanner = new Scanner(new File(_outputFileName), "utf8");
			StringBuilder stringBuilder = new StringBuilder();
			
			while (scanner.hasNextLine())
				stringBuilder.append(scanner.nextLine() + "\n");
			
			return stringBuilder.toString();
			
		} catch (Exception e) {
			e.printStackTrace();
			return "Output file not found";
		}
	}
	
}
