<?php

$b_Num = "";
$b_Name = "";

if(isset($_GET['b_Num'])){
 $b_Num = $_GET['b_Num'];
}
 
$fpath = 'conferioData.csv';
$lines = file($fpath);
$count = count(file($fpath));

$data_json = array();
$data_array = array();

foreach($lines as $line){
 $list = preg_split("/,/", $line);
 $data = array('b_Num'=> $list[0], 'b_ID'=> $list[1], 'b_Name'=> $list[2], 'b_RoomName'=> $list[3],'b_Date'=> $list[4], 'b_StartTime'=> $list[5], 'b_FinishTime'=> $list[6], 'b_Address'=> trim($list[7]));
 $data_json[] = json_encode($data);
};

$i = 0;
$flag = false;
for(;$i < $count; $i++){
 
 $data_array = json_decode($data_json[$i], true);
 //echo $data_json[$i];
 //echo $data_array[b_Num];
 //print($data_array['b_Num']);
 //print("==");
 //print($b_Num);
 if($data_array['b_Num'] == $b_Num){
  $flag = true;
  break;
 }
};

if($flag == true){
 //echo $data_array['b_Name'];
 //header("Content-Type: application/json; charset=utf-8");
 echo $data_json[$i];
} else {
 echo "No Such Number!";
}



?>
