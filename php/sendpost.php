<?php
ini_set("display_errors", 1);
error_reporting(E_ALL);
$url = "http://192.168.10.225/respost.php";
$name = "Hamada";
$data = array(
	'name' => $name,
);

$data = http_build_query($data, "", "&");
$header = array(
	"Content-Type: application/x-www-form-urlencoded",
	"Content-Length: ".strlen($data)
);
$options = array('http' => array(
	'method' => 'POST',
	'header' => implode("\r\n", $header),
	'content' => $data
));
$options = stream_context_create($options);
$contents = file_get_contents($url, false, $options);
?>
