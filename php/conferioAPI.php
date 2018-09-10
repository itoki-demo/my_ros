<?php
#init_set("display_errors", 1);
#error_reporting(E_ALL);
$b_Num = "";
$confRec = "http://192.168.10.136/rapyutaJSON.php?b_Num=";
if(isset($_GET['b_Num'])){
	$b_Num = $_GET['b_Num'];
	$confRec .= $b_Num;
	#echo $confRec;
}
$contents = array();
$contents = file_get_contents($confRec);
echo $contents;
?>
