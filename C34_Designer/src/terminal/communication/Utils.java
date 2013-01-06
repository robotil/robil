package terminal.communication;

import java.util.ArrayList;
import java.util.Random;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Utils {
	
	public static final String componentIdRegex = "\\[id=\\S+\\]"; //[id=....]
	public static final String letters = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	
	private static Random rnd = new Random(System.currentTimeMillis());

	public static ArrayList<String> getMatchedInstances(String txt, String regex) {
		return getMatchedInstances(txt, regex, 4, 1);
	}
	public static ArrayList<String> getMatchedInstances(String txt, String regex, int beginIndex, int endIndex) {
		ArrayList<String> ids = new ArrayList<String>();
		
		Pattern pattern = Pattern.compile(regex);
		Matcher m = pattern.matcher(txt);
		
		while (m.find()) {
			String rawID = m.group(0); 
			ids.add(rawID.substring(beginIndex, rawID.length()-endIndex));
		}
		
		return ids;
	}
	
	public static String randomString(int length) {
		String ret = new String();
		int limit = letters.length();
		
		for (int i = 0; i < length; ++i) {
			int index = rnd.nextInt(limit);
			ret += letters.charAt(index);
		}
		
		return ret;
	}
}
