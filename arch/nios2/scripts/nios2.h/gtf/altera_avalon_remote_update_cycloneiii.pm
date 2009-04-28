package altera_avalon_remote_update_cycloneiii;

require PTF::SystemPTF;
require PTF::SystemPTF::Module;
use strict;

sub run {
  my ($system, $altremote_name) = @_;
  my $module = $system->getModule ($altremote_name);
  my $base_address = $module->getBaseAddress ();
  $base_address = hex ($base_address) | 0x80000000;
# undefine all the old symbols first
    print "#undef na_${altremote_name}\n";

# define base address
    $base_address = sprintf ("%#010x", $base_address);
    printf ("%-41s %30s\n", "#define na_altremote", "${base_address}");
    print "\n";
}

1;
