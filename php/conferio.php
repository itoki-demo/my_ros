<?php
#ini_set("display_errors", 1);
#error_reporting(E_ALL);

$b_Num = "";
$b_Name = "";

if(isset($_GET['b_Num'])){
	$b_Num = $_GET['b_Num'];
}

$fpath = "bookingData.txt";
$lines = file($fpath);
$count = count(file($fpath));

$data_json = array();
$data_array = array();

foreach($lines as $line){
	$list = preg_split("/ /", $line);
	$data = array('b_ID' => $list[0], 'b_Name' => $list[1], 'b_RoomName' => $list[2], 'b_Time' => $list[3], 'b_Num' => trim($list[4]));
	$data_json[] = json_encode($data);
};

$flag = false;
for($i = 0; $i < $count; $i++){
	$data_array = json_decode($data_json[$i], true);
	if($data_array['b_Num'] == $b_Num){
		$flag = true;
		break;
	}
};

$bpath = "room_info.txt";

if($flag == true){
	echo $data_json[$i];

	$fp = fopen($bpath, "w");
	fwrite($fp, $data_array['b_RoomName']);
	fclose();

}else{
	echo "No such number...";
}


?>
