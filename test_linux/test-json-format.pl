#!/usr/bin/perl

# test-json-format.pl
# Copyright 2021 Sébastien Millet

# Read input as JSON5, decode and prints it out.
# If JSON5 data is not correct JSON5 representation, it'll 

use utf8;

use 5.016;

use strict;
use warnings;

use Data::Printer;
use JSON5;

my $str;
{
    local $/ = undef;
    $str = <>;
}

my $h = decode_json5($str);

say "OK";

# vim: ts=4:sw=4:tw=80:et
