<?php
$file = "room_info.txt";
$status = $_GET['room_name'];
file_put_contents($file,$status);
?>

