<?php

/**
 * File: define_strings.php
 * 
 * Output message ids defines from a message catalog
 *
 */

$base_nid = 1;
$undef = false;

for ($i = 1; $i < $_SERVER['argc']; $i++)
{
	$file = $_SERVER['argv'][$i];

	$strings = json_decode(file_get_contents($file), true);

	if ($strings !== false)
	foreach ($strings as $nid => $message)
	{
        echo "#define {$message['id']}_ID {$nid}\n";
	}

	$base_nid = $nid;
}
