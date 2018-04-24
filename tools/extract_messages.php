<?php

/**
 * File: extract_messages.php
 * 
 * Extract message strings from a header file
 * 
 * --- Prototype ---
 * extract_messages.php <header_file> <message_catalog>
 * -----------------
 * 
 * Parameters:
 *  header_file - Header file containing MSG_* defined strings
 *  message_catalog - Message catalog in JSON format
 * 
 */

/* ### Data
 * The format in error_codes.json should be:
 * 
 * array(
 *  "idN" => array(
 *	  "id" => 'string',
 *	  "translations" => array(
 *    	"it": 'string?',
 *	    "en": 'string?'
 *    )
 * 	)
 * )
 */

/**
 * Variable: $language_ids
 * 
 * Map of supported languages codes to ids
 */
$language_ids = array(
	'en' => 1,
	'it' => 7
);

/**
 * Function: load_strings
 * 
 * Load message defines from a catalog file
 */
function load_strings ($catalog_file_name)
{
	global $language_ids;

	$catalog = json_decode(file_get_contents($catalog_file_name), true);

	// Validate catalog structure
	// Catalog must be an array
	if (!is_array($catalog)) return false;

	// In each Catalog item
	foreach ($catalog as $id => $message)
	{
		// Id must be numeric
		if (!is_numeric($id)) return false;
		// Message must contain 'id' key
		if (!array_key_exists('id', $message)) return false;
		// Message must contain 'transaltions' key
		if (!array_key_exists('translations', $message)) return false;

		foreach ($message['translations'] as $lang => $trans) {
			// Lang must be e mapped language
			if (!in_array($lang, array_keys($language_ids))) return false;
		}
	}

	return $catalog;
}

/**
 * Function: collect_strings
 * 
 * Scan a header file, extracting message strings from a specifed language
 */
function collect_strings ($header_file, $lang)
{
	global $language_ids;
	if (!is_numeric($lang))
		$lang = $language_ids[$lang];

	$messages = array();

	$_lines = file($header_file);
	$_current_lang = null;
	foreach ($_lines as $line)
	{
		// If we are in a language block attemp at parsing the messages
		if ($_current_lang == $lang)
		{
			// Unless we encounter an #endif
			if (preg_match('/^\s*\#endif\s*$/', $line)) {
				$_current_lang = null;
				continue;
			}

			// record message id and string
			if (preg_match('/^\s*#define\s+(MSG_[0-9A-Z_]+)\s+"([^"]*)"\s*$/', $line, $match)) {
				$id = $match[1];
				$string = $match[2];
				$messages[$id] = $string;
			}
			continue;
		}

		// Look for language start
		if (preg_match('/^\s*\#if\s+LANGUAGE_CHOICE\s*==\s*(\d+)\s*$/', $line, $match)) {
			$_current_lang = $match[1];
		}
	}

	return $messages;
}

/* ### I/O
 * The input and output files are
 * 
 * string header => languages.h
 * 
 * string catalog => error_codes.json
 */

$header_file_name = 'languages.h';
switch ($_SERVER['argc'])
{
	case 3:
		$catalog_file = $_SERVER['argv'][2];
	case 2:
		$header_file_name = $_SERVER['argv'][1];
}

/**
 * This script should:
 * 
 * 1. load message defines from error_codes.json
 * 
 * 2. scan langauges.h and collect string ids and values
 * 
 * 3. add entries in error_codes.json for missing string ids
 */

if (isset($catalog_file))
{
	if (!file_exists($catalog_file))
		file_put_contents($catalog_file, '[]');

	$catalog = load_strings($catalog_file);

	if ($catalog === false) die("Wrong catalog format in: {$catalog_file}");
}
else
{
	$catalog = array();
}

foreach ($language_ids as $lang => $id)
{
	$messages = collect_strings($header_file_name, $id);

	// Update existing messages
	$_last_id = 0;
	foreach ($catalog as $id => &$message)
	{
		if ($id > $_last_id)
			$_last_id = $id;
		if (array_key_exists($message['id'], $messages)) {
			$message['translations'][$lang] = $messages[$message['id']];
			unset($messages[$message['id']]);
		}
	}

	// Add missing messages
	foreach ($messages as $id => $string)
	{
		$catalog[++$_last_id] = array(
			'id' => $id,
			'translations' => array(
				$lang => $string
			)
		);
	}
}

$json_catalog = json_encode($catalog, JSON_PRETTY_PRINT);
if (isset($catalog_file)) {
	file_put_contents($catalog_file, $json_catalog);
} else {
	echo $json_catalog;
}
