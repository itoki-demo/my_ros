<?php
//ini_set("display_errors", 1);
//error_reporting(E_ALL);
$fpath = "status.txt";
if(isset($_GET['status'])){
	$status = $_GET['status'];
	echo " isSet";
	if(file_exists($fpath)){
		echo " is exists";
		$fpath = fopen($fpath, "w");
		fwrite($fpath, $status);
		fclose($fpath);
		echo "sakusesu!";
	}
}else{
	echo "No status...";
}

?>
