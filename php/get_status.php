<?php
$fpath = "status.txt";
if(file_exists($fpath)){
	$fpath = fopen($fpath, "r");
	$status = fgets($fpath);
	fclose($fpath);
	echo $status;
}else{
	echo "can't find " . $fpath;
}
?>
