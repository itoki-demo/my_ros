<?php
$fpath = "status.txt";
$fpath = fopen($fpath, "w");
fwrite($fpath, "navigation");
fclose($fpath);
?>
