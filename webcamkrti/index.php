<?php

/*
ZoneMinder Event Viewer
Copyleft: Luca Tringali - G.S. Talpe del Carso
Place this file in the same folder where ZoneMinder uploads zip files of monitor events.
*/

echo '<h3>Foto dalla webcam delle Talpe del Carso</h3>';
extractZipfiles();



echo "These images are released by G.S. Talpe del Carso under Creative Commons Attribuzione 3.0 licence <a href=\"https://creativecommons.org/licenses/by/3.0/it/\"><img src=\"https://mirrors.creativecommons.org/presskit/buttons/88x31/png/by.png\" height=\"25\"/></a>";

print("<div id=\"options\">");
print("<form action=\"index.php\">");
print("Reverse <input type=\"checkbox\" name=\"reverse\" value=\"reversed\"><br>");
print("Animation <input type=\"checkbox\" name=\"animation\" value=\"animation\"><br>");
print("Results number: <input type=\"text\" name=\"max\"><br>");
print("<input type=\"submit\" value=\"Submit\"></form>");
print("</div>");

print('<script>');
print('var imageIndex = 0;'); 
print('</script>');

$lastdir = '';
$thumbnailh = '120';
$animationh = '480';
$animinterval = '500';

if ($dhandle = opendir('.')) {
    
    $max = $_GET['max'];
    if ($max == '') $max = 100;
    $reverse = $_GET['reverse'];
    $animation = $_GET['animation'];
    
    $i = 0;
    $dhandle2 = opendir('.');
    while (false !== ($dentry2 = readdir($dhandle2))) {
        if ($dentry2 != "." && $dentry2 != ".." && $dentry2 != "index.php" && $dentry2 != "old" && strpos($dentry2, '.') === false) {
            if(is_dir("./".$dentry2)) {
                $lastdir = $dentry2;
                $i = $i + 1;
                if ($i>$max) break;
            }
        }
    }
    //print($lastdir);
    $i = 0;
    while (false !== ($dentry = readdir($dhandle))) {
        if ($dentry != "." && $dentry != ".." && $dentry != "index.php" && $dentry != "old" && strpos($dentry, '.') === false) {
            if(is_dir("./".$dentry)) {
                echo "<b>".$dentry."</b></br>";
                if ($handle = opendir("./".$dentry)) {
                    
                    $files = array();
                    while (false !== ($entry = readdir($handle))) {
                        if ($entry != "." && $entry != ".." && $entry != "index.php") {
                            if(is_file($dentry."/".$entry)) $files[] = $dentry."/".$entry;
                        }
                    }
                    closedir($handle);
                    
                    if ($reverse != '') {
                        sort($files);
                        $files = array_reverse($files);
                    } else {
                        sort($files);
                    }                    
                    
                    $n = 0;
                    if ($animation != '') {
                        $strdentry = str_replace("-","",$dentry);
                        print( '<img height="'.$animationh.'" id= "'.$strdentry.'Image" />' );
                        print('<script>');
                        print('var imageArray'.$strdentry.' = new Array();');
                    }
                    foreach ($files as $entry) {
                        if (filesize($entry) > 1024) {
                            if ($animation == '') {
                                echo '<a href="'.$entry.'">';
                                if ( substr($entry, -4)=='.jpg' ) echo '<img height="'.$thumbnailh.'" src="'.$entry.'"/>';
                                echo '</a>';
                            } else {
                                if ( substr($entry, -4)=='.jpg' ) {
                                    $strdentry = str_replace("-","",$dentry);
                                    print('imageArray'.$strdentry.'.push( "'.$entry.'" );');
                                    $n = $n + 1;
                                }
                            }
                        }
                        
                    }
                    if ($animation != '') {
                        $strdentry = str_replace("-","",$dentry);
                        $nextimg = '';
                        if ($dentry == $lastdir) $nextimg = 'imageIndex = imageIndex + 1;';
                        print(' setInterval( function() { var myImage = document.getElementById("'.$strdentry.'Image");  thisimageIndex = imageIndex % imageArray'.$strdentry.'.length;  myImage.setAttribute("src",imageArray'.$strdentry.'[thisimageIndex]); '.$nextimg.'}, '.$animinterval.' );');
                        print('</script>');
                    }   
                }
                $i = $i + 1;
            }
            echo '</br>';
            if ($i>$max) break;
        }
    }
}





function extractZipfiles()
{
if ($handle = opendir('.')) {

    $files = array();
    while (false !== ($entry = readdir($handle))) {
        if ($entry != "." && $entry != ".." && $entry != "index.php") {
            if(is_file($entry)) $files[] = $entry;
        }
    }
    closedir($handle);
    
    $files = array_reverse($files);
    
    $i = 0;
    foreach ($files as $entry) {
        if (filesize($entry) > 1024) {
        if ( substr($entry, -4)=='.zip' ){
            $mytree = getTree($entry);
            if (count($mytree)>0) {
                $folder = $mytree[0];
                // usr/share/zoneminder/events/2/19/01/06/13/19/15/001-capture.jpg 
                $folder = preg_replace("/\/[0-9]*-capture.*/", "", $folder);
                $folder = preg_replace("/.*events\/[0-9]*\//", "", $folder);
                $folder = preg_replace("/\//", "-", $folder);
                $folder = preg_replace("/-.*/", "_", $entry).$folder;
                
                mkdir($folder);
                foreach ($mytree as $fileinzip) {
                    extractFile($entry, $fileinzip, $folder);
                    $basename = preg_replace("/.*\//", "", $fileinzip);
                    echo '<a href="'.$folder."/".$basename.'">';
                    echo $folder."/".$basename;
                    echo '</a></br>';
                }
                rename($entry, $entry."OLD");
            }
        }
        $i = $i + 1;
        } else {
        unlink($entry);
        }
        //if ($i>$max) break;
    }

}
}

function getTree($zippath) 
{
$zip = zip_open($zippath); 
$dirs = $files = array();

if ($zip) {
    while ($zip_entry = zip_read($zip)) {
        $zen = zip_entry_name($zip_entry);
        $is_dir = substr($zen, -1) == DIRECTORY_SEPARATOR;

        $zen_splitted = explode(DIRECTORY_SEPARATOR, $zen);
        if ($is_dir) {
            $dirs[]  = $zen_splitted[count($zen_splitted)-2];
        } else {
            $files[]  = $zen;//$zen_splitted[count($zen_splitted)-1];
        }
    }

    zip_close($zip);

}
return $files;
}

function extractFile($zippath, $file, $folder)
{

$fileData = '';
$zp = new ZipArchive();

if ($zp->open($zippath)) {
    $fp = $zp->getStream($file);
    if(!$fp) exit("failed\n");
    while (!feof($fp)) {
        $fileData .= fread($fp, 8192);
    }
    fclose($fp);
    $basename = preg_replace("/.*\//", "", $file);
    file_put_contents($folder."/".$basename, $fileData);
}
}


?>
