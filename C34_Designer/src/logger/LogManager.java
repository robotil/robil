package logger;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Scanner;

public final class LogManager {

	private static String _outputFileName;
	private static PrintStream _outputStream;
	
	private static PrintStream _stdOutStream;
	private static PrintStream _stdErrStream;
	
	private static boolean _outputRedirected = false;

	static {
		_stdOutStream = System.out;
		_stdErrStream = System.err;
	}
	
	private static PrintStream createPrintStreamToFile(String fileName) {
		try {
			return new PrintStream(new BufferedOutputStream(
					new FileOutputStream(fileName)));
		} catch (FileNotFoundException e) {
			Log.e(e);
			return null;
		}
	}

	public static class LineCounter{
		public long number;
		public LineCounter(long n) {
			number = n;
		}
	}
	public static String getOutputContent(LineCounter fromline) {
		if (_outputFileName == null)
			return "Output filename not set";

		if (_outputStream == null)
			return "Output stream not set";

		_outputStream.flush();

		Scanner scanner = null;
		
		try {
			scanner = new Scanner(new File(_outputFileName), "utf8");
			StringBuilder stringBuilder = new StringBuilder();

			long linen =0;
			while (scanner.hasNextLine()){
				String line = scanner.nextLine();
				if(linen >= fromline.number)
					stringBuilder.append(line + "\n");
				linen+=1;
			}
			fromline.number = linen;

			return stringBuilder.toString();

		} catch (Exception e) {
			Log.e(e);
			return "Output file not found";
		} finally {
			scanner.close();
		}
	}

	public static String getOutputFileName() {
		return _outputFileName;
	}

	public static void redirectStandardAndErrorOutput(String outputFileName) {
		System.out.println("Redirecting all output to " + new File(outputFileName).getAbsolutePath());
		_outputFileName = outputFileName;
		_outputStream = createPrintStreamToFile(outputFileName);

		System.setOut(_outputStream);
		System.setErr(_outputStream);
		System.out.println("Redirecting all output to " + new File(outputFileName).getAbsolutePath());
		
		_outputRedirected = true;
	}
	
	public static PrintStream getCurrentOutputStream() {
		return System.out;
	}
	
	public static PrintStream getCurrentErrorStream() {
		return System.err;
	}
	
	public static PrintStream getStdOutputStream() {
		return _stdOutStream;
	}
	
	public static PrintStream getStdErrorStream() {
		return _stdErrStream;
	}
	
	public static boolean isRedirected() {
		return _outputRedirected;
	}

}
