<?php

#ini_set("display_errors", 1);
#error_reporting(E_ALL);
$b_RoomName = $_GET['b_RoomName'];
$rpath = "room_info.txt";
#echo $b_RoomName;
if(file_exists($rpath)){
		$rpath = fopen($rpath, "w");
		fwrite($rpath, $b_RoomName);
		fclose($rpath);
}
?>
