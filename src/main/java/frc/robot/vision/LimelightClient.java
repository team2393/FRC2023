// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Helper for dealing with Limelight */
public class LimelightClient
{
  /*
  Network table "limelight-front", "json": "t6t_rs" contains the x, y, z position
  {"Results":{"Classifier":[],"Detector":[],"Fiducial":[{"fID":6,"fam":"16H5C","pts":[],"skew":[],"t6c_ts":[-0.1819821297174766,0.19152947927501754,-1.2927626167319477,9.536598317113642,5.656188787485489,0.8302258066244446],"t6r_fs":[-5.950337624721007,0.2342261550517298,0.2711905207249825,0.837496139143989,-9.45377348720073,174.20548791505757],"t6r_ts":[-0.1819821297174766,0.19152947927501754,-1.2927626167319477,9.536598317113642,5.656188787485489,0.8302258066244446],"t6t_cs":[0.050902202415996876,0.024601799378643218,1.3182715719623306,-9.501442252719597,-5.715398671275721,0.11734407372542792],"t6t_rs":[0.050902202415996876,0.024601799378643218,1.3182715719623306,-9.501442252719597,-5.715398671275721,0.11734407372542792],"ta":0.011568170972168446,"tx":2.081610679626465,"txp":169.65443420410156,"ty":-1.1051111221313477,"typ":124.498291015625}],"Retro":[],"botpose":[-5.950337201950147,0.2342294161416599,0.27119060041452736,0.8374914064004849,-9.453768731433065,174.20563327242846],"pID":1.0,"tl":24.398488998413086,"ts":512294.35746,"v":1}}
  */

  public static void main(String[] args) throws Exception
  {
    String json = Files.readString(Path.of(LimelightClient.class.getResource("json.txt").toURI()));
    System.out.println(json);

    ObjectMapper mapper = new ObjectMapper();

    // HashMap<String, String> data = mapper.readValue(json, HashMap.class);
    // System.out.println(data);

    JsonNode node = mapper.readTree(json);
    System.out.println(node.get("Results")
                           .get("Fiducial")
                           );
  }
}
