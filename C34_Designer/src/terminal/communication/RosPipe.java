package terminal.communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import logger.Log;

import document.Parameters;

import terminal.lineprocessors.LineProcessor;

public class RosPipe {

	public enum RosTargets {
		Topic, Service, Rossrv
	}

	protected Process process;
	private RosTargets target;
	private LineProcessor processor;
	private String[] args;

	private Thread thread;

	public RosPipe(Thread thread, RosTargets target, LineProcessor processor,
			String... args) {
		this.target = target;
		this.processor = processor;
		this.args = args;
		this.thread = thread;
	}

	private String convertRosEnumToString(RosTargets target) {
		String roscommand = null;
		switch (target) {
		case Topic:
			roscommand = "rostopic";
			break;
		case Service:
			roscommand = "rosservice";
			break;
		case Rossrv:
			roscommand = "rossrv";
			break;
		}

		return roscommand;
	}

	public boolean isAlreadyRunning() {
		return this.process != null;
	}

	private void printArgArray(List<String> args) {
		String result = "Arg: ";
		for (String string : args) {
			result = result + string + " ! ";
		}

		Log.e(result);
	}

	private void receive(RosTargets target, String streamid, LineProcessor processor )
			throws IOException {

		if (this.process == null) {
			return;
		}

		if (this.thread.isInterrupted()) {
			return;
		}

		BufferedReader process_stdout = new BufferedReader(
				new InputStreamReader(this.process.getInputStream()));

		String line = null;
		processor.onStart();
		try {
			while ((line = process_stdout.readLine()) != null) {
				if (this.thread.isInterrupted()) {
					return;
				}
				if(Parameters.log_print_ros_output)
					Log.d("new line from ros stream ("+streamid+") "+convertRosEnumToString(target)+": "+line);
				processor.onNewLine(line);
			}
		} catch (IOException e) {
			throw new IOException(String.format("Reading problem %s: %s",
					convertRosEnumToString(target), e.getMessage()));

		} finally {
			processor.onEnd();
			this.process = null;
		}
	}

	private void send(RosTargets target, String stream_id, String... args) throws IOException {
		String roscommand = convertRosEnumToString(target);

		@SuppressWarnings("unused")
		int emptyCounter = 0;
		for (String s : args) {
			if (s.trim().equals(""))
				emptyCounter++;
		}

		// build command arguments
		ArrayList<String> arg_list = new ArrayList<String>();
		arg_list.add(roscommand);

		// int j = 1;
		for (int i = 0; i < args.length; ++i) {
			if (args[i].trim().equals("") == false)
				arg_list.add(args[i]);
		}
		
		if(Parameters.log_print_ros_commands){
			System.out.print("Ros command ("+stream_id+") ");
			printArgArray(arg_list);
		}
		ProcessBuilder pb = new ProcessBuilder(arg_list);

		try {
			this.process = pb.start();
		} catch (IOException e) {
			this.process = null;

			String errorMsg = String.format("Cannot open %s: %s", roscommand,
					e.getMessage());
			throw new IOException(errorMsg);
		}
	}

	public void sendAndReceive() throws IOException {
		sendAndReceive(this.target, this.processor, this.args);
	}

	static long stream_id_counter=0;
	private void sendAndReceive(RosTargets target, LineProcessor processor,
			String... args) throws IOException {
		
		stream_id_counter++;
		Long sid = new Long(stream_id_counter);

		if (this.thread.isInterrupted()) {
			return;
		}
		if(Parameters.log_ros_progress_print_level > 1)
			Log.d("   send message to ros.");
		send(target, sid.toString(), args);

		if (this.thread.isInterrupted()) {
			return;
		}

		if(Parameters.log_ros_progress_print_level > 1)
			Log.d("   listen output stream from ros.");
		receive(target, sid.toString(), processor);

	}

	public void stop() {
		if (this.process != null) {
			this.process.destroy();
		}
	}
}
