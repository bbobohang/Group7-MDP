package com.example.demo;

import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.ResponseBody;
import org.springframework.web.bind.annotation.RestController;


@SpringBootApplication
@RestController
@ResponseBody
@Controller
public class DemoApplication {
	
	public static void main(String[] args) {
		// GUI running here, comment out if dw
		main test = new main();
		String json = "{\"cat\":\"obstacles\",\"value\":{\"obstacles\":[{\"d\":6,\"x\":5,\"y\":13,\"id\":2},{\"d\":2,\"x\":15,\"y\":15,\"id\":0}, {\"d\":0,\"x\":10,\"y\":15,\"id\":0}, {\"d\":2,\"x\":15,\"y\":7,\"id\":2},{\"d\":6,\"x\":5,\"y\":7,\"id\":2}],\"mode\":\"0\"}}";
		String outputjson = test.findPathJson(json);
		
		SpringApplication.run(DemoApplication.class, args);
		
	}

	// @RequestMapping("/response")
	// public Map<String, String> sayHello() {
	// 	HashMap<String, String> map = new HashMap<>();
	// 	map.put("key", "value");
	// 	map.put("foo", "bar");
	// 	map.put("aa", "bb");
	// 	return map;
	// }

	@PostMapping("/api")
    public String postBody(@RequestBody JSONObject obstacles) {

		main test = new main();
		String output = test.callAlgo(obstacles);

        return output;
    }
	
	@PostMapping("/checklist")
    public String postBody(@RequestBody String obstacles) {
		main test = new main();
		String output = test.turnToNextSide(obstacles);
        return output;
    }
}

