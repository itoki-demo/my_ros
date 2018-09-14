<?php
#ini_set("display_errors", 1);
#error_reporting(E_ALL);
$fpath = 'conf_info.json';
if(file_exists($fpath)){
 $json = file_get_contents($fpath);
 $json = mb_convert_encoding($json, 'UTF8', 'ASCII,JIS,UTF-8,EUC-JP,SJIS-WIN');
}
echo $json;

$conf_init = "http://192.168.20.42/update_conf_info.php"
file_get_contents($conf_init)
?>
