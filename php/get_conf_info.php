<?php
#ini_set("display_errors", 1);
#error_reporting(E_ALL);
$fpath = 'conf_info.json';
if(file_exists($fpath)){
 $json = file_get_contents($fpath);
 $json = json_decode($json);
 $json = json_encode($json, JSON_UNESCAPED_UNICODE);

}
echo $json;

$conf_init = "http://192.168.20.42/update_conf_info.php";
file_get_contents($conf_init);
?>
