<?php
$temperature = $_GET['temperature'];
$humidity = $_GET['humidity'];
$pressure = $_GET['pressure'];
$txt = date("Y-m-d H:i:s")." TEMPERATURE: ".$temperature." PRESSURE: ".$pressure." HUMIDITY: ".$humidity;
$myfile = file_put_contents('logs.txt', $txt.PHP_EOL , FILE_APPEND | LOCK_EX);  
?>

