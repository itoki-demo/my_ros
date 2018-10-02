<?php
$fpath = "reservation_number.txt";
if(file_exists($fpath)){
	$fpath = fopen($fpath, "r");
	$status = fgets($fpath);
	fclose($fpath);
	echo $status;
}else{
	echo "can't find " . $fpath;
}
$reserve_file = fopen($fpath, "w");
fwrite($fpath, 'no input');
fclose($);
?>
