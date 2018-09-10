<?php
#ini_set("display_errors", 1);
#error_reporting(E_ALL);
$fpath = 'room_info.txt';
$lines = file($fpath);

foreach($lines as $line){
	$roomname = $line;
}

echo $roomname;

$fpath = fopen($fpath, "w");
fwrite($fpath, "no goal");
fclose($fpath);

?>
