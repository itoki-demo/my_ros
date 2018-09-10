<?php
echo "aiueo";
echo $_POST['name'];
$fpath = fopen("post.txt", "w");
fwrite($fpath, $_POST['name']);
fclose();
?>
