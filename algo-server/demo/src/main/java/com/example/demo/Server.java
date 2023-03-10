package com.example.demo;

import java.net.*;
import java.nio.charset.StandardCharsets;
import java.text.ParseException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

import java.io.*;
import java.time.*;
import java.time.format.DateTimeFormatter;

public class Server {
	public static void writeToFile(String message) throws IOException {
		
		JSONObject jsonObject = (JSONObject) JSONValue.parse(message);  
		 
		JSONArray obstaclesArr = (JSONArray) jsonObject.get("obstacles");
		
		
		LocalDateTime now = LocalDateTime.now();
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
        String formattedDateTime = now.format(formatter);
        String fileName = formattedDateTime + ".txt";
        File file = new File(fileName);
        file.createNewFile();
        FileWriter fw = new FileWriter(file.getAbsoluteFile());
        BufferedWriter bw = new BufferedWriter(fw);
        bw.write(message + "\n");
        bw.write("\n" + obstaclesArr.toString().replace('"', '\"'));
        bw.close();
	}
	
	public static void main(String[] args) throws IOException {
	      try {
	       ServerSocket serversocket = new ServerSocket(6000);
	       System.out.println("Waiting for connections");
	       Socket client = serversocket.accept();
	       main test = new main();
	       
	       final BufferedReader input = new BufferedReader(
	           new InputStreamReader(client.getInputStream(), StandardCharsets.UTF_8)); // changed
	       final OutputStream output = client.getOutputStream();
	 
	        String message = input.readLine(); // changed
	        System.out.println(message);
	        writeToFile(message);
	        test.createGUI();
	        String x = test.findPathJson(message);
	        output.write(x.getBytes(StandardCharsets.UTF_8)); // changed
	 
	       //client.close();
	       //serversocket.close();
	 
	         } catch (IOException e) {
	             e.printStackTrace();
	         } 
	 //     catch (org.json.simple.parser.ParseException e) {
	 //   // TODO Auto-generated catch block
	 //   e.printStackTrace();
	 //  }
	       //System.exit(0);
	     }
}
