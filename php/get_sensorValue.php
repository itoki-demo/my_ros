<?php
//ini_set("display_errors", 1);
//error_reporting(E_ALL);

$s = array ( "humidity" => array ( array ( "date" => "kyou", "value" => "0%"),
								   array ( "date" => "kyou", "value" => "1%"),
								   array ( "date" => "kyou", "value" => "2%"),
								   array ( "date" => "kyou", "value" => "3%"),
								   array ( "date" => "kyou", "value" => "4%"),
								   array ( "date" => "kyou", "value" => "5%"),
								   array ( "date" => "kyou", "value" => "6%"),
								   array ( "date" => "kyou", "value" => "7%"),
								   array ( "date" => "kyou", "value" => "8%"),
								   array ( "date" => "kyou", "value" => "9%")
						   ),
			  "temperature" => array ( array ( "date" => "kyou", "value" => "0.0"),
								   		array ( "date" => "kyou", "value" => "1.1"),
								   		array ( "date" => "kyou", "value" => "2.2"),
								   		array ( "date" => "kyou", "value" => "3.3"),
								   		array ( "date" => "kyou", "value" => "4.4"),
								   		array ( "date" => "kyou", "value" => "5.5"),
								   		array ( "date" => "kyou", "value" => "6.6"),
								   		array ( "date" => "kyou", "value" => "7.7"),
								   		array ( "date" => "kyou", "value" => "8.8"),
								   		array ( "date" => "kyou", "value" => "9.9")
						   		)
);
$humidity = "";
$temperature = "";
$fpath = "data.txt";
$json_data = file_get_contents($fpath);
$sensor_data = json_decode($json_data, true);
//unset($sensor_data['temperature'][8]);
//unset($sensor_data['humidity'][8]);
$json_data = json_encode($s);
//echo $sensor_data['temperature'];
//echo $sensor_data['humidity']['2018/07/30 10:00:47'];
//var_dump($json_data);
echo $json_data
?>
