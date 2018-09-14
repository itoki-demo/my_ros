<?php
$file = "conf_info.json";
$key_array = array('b_Num','b_ID','b_Name','b_RoomName','b_Date','b_StartTime','b_FinishTime');
$data_json = array();
foreach($key_array as $key){
 $data_json += array($key=>$_GET[$key]);
}
$data_json = json_encode($data_json);
file_put_contents($file,$data_json);
?>

