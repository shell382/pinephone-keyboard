#!/usr/bin/env php
<?php

$pmap = file_get_contents("physical-map.txt");
$kmap = file_get_contents("factory-keymap.txt");

// high nibble = row, low nibble col
$el_phys_map = [];
foreach (explode("\n", $pmap) as $ln) {
	if (preg_match('#^\s*(\d+):(\d+)\s+(\d+):(\d+)#', $ln, $m)) {
		$pp = (((int)$m[1] << 4) | (int)$m[2]);
		$ep = ((((int)$m[3] - 1) << 4) | ((int)$m[4] - 1));
		$el_phys_map[$ep] = $pp;
	}
}

//var_export($el_phys_map);

echo "static const uint8_t el_phys_map[256] = {\n\t";
for ($i = 0; $i < 256; $i++) {
	if ($i > 0 && $i % 8 == 0)
		echo "\n\t";
	echo sprintf("0x%02x, ", $el_phys_map[$i] ?? 0xff);
}
echo "\n};\n\n";

$key_names = [];

$phys_key_base = [];
$phys_key_fn = [];
$phys_key_pine = [];

foreach (explode("\n", $kmap) as $ln) {
	if (preg_match('#^\s*(\d+):(\d+)\s+([^\#]+)#', $ln, $m)) {
		$pp = (((int)$m[1] << 4) | (int)$m[2]);

		$alts = preg_split('#\s+#', $m[3], -1, PREG_SPLIT_NO_EMPTY);
		
		for ($i = 0; $i < 3; $i++) {
			$alt = $alts[$i] ?? null;
			if (!$alt)
				break;

			$keys = preg_split('#\+#', $alt, -1, PREG_SPLIT_NO_EMPTY);
			$key_names = array_merge($key_names, $keys);
	
			if ($i == 0)		
				$phys_key_base[$pp] = $keys;
			if ($i == 1)		
				$phys_key_fn[$pp] = $keys;
			if ($i == 2)		
				$phys_key_pine[$pp] = $keys;

//			echo "$pp $i ".implode(" ", $keys)."\n";
		}

		$key_map[$pp] = $m[3];
	}
}

$key_names = array_unique($key_names);
echo "static const int used_keys[] = {\n";
foreach ($key_names as $n)
	echo "\tKEY_$n,\n";
echo "};\n\n";

echo "static const char* key_names[] = {\n";
foreach ($key_names as $n)
	echo "\t[KEY_$n] = \"$n\",\n";
echo "};\n\n";

function kmap_to_code($name, $map) {
	echo "static const int {$name}[256][2] = {\n";
	for ($i = 0; $i < 256; $i++) {
		if ($map[$i] ?? null)
			echo sprintf("\t[0x%02x] = { %s },\n", $i, implode(", ", array_map(function($n) { return "KEY_$n"; }, $map[$i])));
	}
	echo "};\n\n";
}

kmap_to_code("keymap_base", $phys_key_base);
kmap_to_code("keymap_fn", $phys_key_fn);
kmap_to_code("keymap_pine", $phys_key_pine);
