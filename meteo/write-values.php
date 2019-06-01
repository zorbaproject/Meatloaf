<?php
// write-values.php?temperature=25&pressure=1013&humidity=30&rain=12.5
$temperature = $_GET['temperature'];
$humidity = $_GET['humidity'];
$pressure = $_GET['pressure'];
$rain = $_GET['rain'];
$txt = date("Y-m-d H:i:s").",".$temperature.",".$pressure.",".$humidity.",".$rain;
$myfile = file_put_contents('logs.txt', $txt.PHP_EOL , FILE_APPEND | LOCK_EX);  
?>

