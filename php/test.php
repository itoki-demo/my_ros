<?php
ini_set("display_errors", 1);
error_reporting(E_ALL);
$str = "";
$contents = json_decode(file_get_contents("http://192.168.10.140/rapyutaJSON.php?b_Num=1111"));

var_dump($contents);
?>
