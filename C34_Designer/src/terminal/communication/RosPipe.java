package terminal.communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import terminal.lineprocessors.LineProcessor;

public class RosPipe {

	protected Process process;

	private RosTargets target;
	private LineProcessor processor;
	private String[] args;
	private Thread thread;

	public enum RosTargets {
		Topic, Service, Rossrv
	}

	public RosPipe(Thread thread, RosTargets target, LineProcessor processor,
			String... args) {
		this.target = target;
		this.processor = processor;
		this.args = args;
		this.thread = thread;
	}

	public void sendAndReceive() throws IOException {
		sendAndReceive(target, processor, args);
	}

	private void sendAndReceive(RosTargets target, LineProcessor processor,
			String... args) throws IOException {

		if (thread.isInterrupted()) {
			return;
		}
		send(target, args);

		if (thread.isInterrupted()) {
			return;
		}

		receive(target, processor);

	}

	private void receive(RosTargets target, LineProcessor processor)
			throws IOException {

		if (process == null) {
			return;
		}

		if (thread.isInterrupted()) {
			return;
		}

		BufferedReader process_stdout = new BufferedReader(
				new InputStreamReader(process.getInputStream()));

		String line = null;
		processor.onStart();
		try {
			while ((line = process_stdout.readLine()) != null) {
				if (thread.isInterrupted()) {
					return;
				}
				processor.onNewLine(line);
			}
		} catch (IOException e) {
			throw new IOException(String.format("Reading problem %s: %s",
					convertRosEnumToString(target), e.getMessage()));

		} finally {
			processor.onEnd();
			process = null;
		}
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

	private void printArgArray(List<String> args) {
		String result = "Arg: ";
		for (String string : args) {
			result = result + string + "!";
		}

		System.err.println(result);
	}

	private void send(RosTargets target, String... args) throws IOException {
		String roscommand = convertRosEnumToString(target);

		int emptyCounter = 0;
		for (String s : args)
		{
			if (s.trim().equals(""))
				emptyCounter++;
		}

		// build command arguments
		ArrayList<String> arg_list = new ArrayList<String>();
		arg_list.add(roscommand);

		int j = 1;
		for (int i = 0; i < args.length; ++i) {
			if (args[i].trim().equals("") == false)
				arg_list.add(args[i]);
		}
		printArgArray(arg_list);
		ProcessBuilder pb = new ProcessBuilder(arg_list);

		try {
			process = pb.start();
		} catch (IOException e) {
			process = null;

			String errorMsg = String.format("Cannot open %s: %s", roscommand,
					e.getMessage());
			throw new IOException(errorMsg);
		}
	}

	public boolean isAlreadyRunning() {
		return process != null;
	}

	public void stop() {
		if (process != null) {
			process.destroy();
		}
	}
}
