<?php

$slackToken = 'xoxp-377336605696-379115517895-413438816112-8eab2fda0aecccc3aff0fbe316b1ac80';
$slackToken = "token=" . $slackToken;
$text = "3kai matigaetayo.";
$text = "text=" . urlencode($text);
$url = "https://slack.com/api/chat.postMessage?";
$channel = "%23test";
$channel = "channel=" . $channel;
$username = "Yuuki";
$username = "username=" . urlencode($username);

$url = $url . $slackToken . "&" . $channel . "&" . $text . "&as_user=true";

file_get_contents($url);

#echo $url;

?>
