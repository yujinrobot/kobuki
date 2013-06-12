<html>
  <head>
    <title>Kobuki Firmware</title>
  </head>
  <body>
  <?php
    //path to directory to scan
    $directory = "./";
 
    //get all image files with a .hex extension.
    $binaries = glob($directory . "*.hex");
 
    //print each file name
    echo("<h1>Firmware for Kobuki</h1>");
    echo("<p>The following files are firmware binaries for the Kobuki. Refer to the web-site ");
    echo("<a href='http://kobuki.yujinrobot.com/documentation/howtos/upgrading-firmware/'>howto</a> ");
    echo("for installation instructions.");
    echo("<ul>\n");
    echo("  <li><a href='./MD5SUMS'>MD5SUMS</a></li>\n");
    foreach($binaries as $binary)
    {
      $name = substr(${binary},2);
      if ( is_link(${binary}) ) {
        $link_name = substr(readlink(${binary}),16,-4);
        echo("  <li><a href='./${binary}'>${name}</a> -> ${link_name}</li>\n");
      } else { 
        echo("  <li><a href='./${binary}'>${name}</a></li>\n");
      }
    }
    echo("</ul>\n");
    echo("<p>Some history:</p>");
    echo("<ul>\n");
    echo("  <li><a href='https://github.com/yujinrobot/kobuki/blob/hydro-devel/kobuki_driver/firmware_changelog.md'>Changelog</a></li>\n");
    echo("</ul>\n");
    echo("<p>About the firmware versions:</p>");
    echo("<ul>\n");
    echo("<li>'factory': this is flashed onto the Kobukis in the factory. It has undergone serious testing, but is usually not the most recent one.</li>\n");
    echo("<li>'latest-stable': more recent, but less tested
</li>\n");
    echo("<li>'latest': most recent, but be aware that this version hasn't been tested much and might not be very stable.</li>\n");
    echo("</ul>\n");
  ?>
  </body>
</html>

