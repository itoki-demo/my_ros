<?php
#ini_set("display_errors", 1);
#error_reporting(E_ALL);

$b_Num = "";
$confRec = "http://192.168.20.45/conferioAPI.php?b_Num=";
$rpath = "room_info.txt";
if(isset($_GET['b_Num'])){
	$b_Num = $_GET['b_Num'];
	$confRec .= $b_Num;
	#echo $confRec;
}
$contents = array();
$contents = file_get_contents($confRec);
echo $contents;

if(file_exists($rpath)){
	$data_json = json_decode($contents, true);
	$rpath = fopen($rpath, "w");
	fwrite($rpath, $data_json['b_RoomName']);
	fclose();
}

?>
