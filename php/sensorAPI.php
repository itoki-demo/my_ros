<?php
$json_string = file_get_contents('php://input');
$log = fopen("data.txt", "w");
flock($log, LOCK_EX);
fputs($log, $json_string);
flock($log, LOCK_UN);
fclose($log);
?>
