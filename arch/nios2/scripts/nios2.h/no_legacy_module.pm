package no_legacy_module;

use base qw(BasicModule);
use strict;

sub required_class_name {
  "no_legacy_module";
}

sub run {
  my ($system, @no_legacy_module_names) = @_;

  foreach my $no_legacy_module_name (@no_legacy_module_names) {
    my $module = $system->getModule ($no_legacy_module_name);
    my $gtf_class_name = $module->getGtfClass ();

    foreach my $dir (@INC) {
    if (-e "$dir/nios2.h/gtf/$gtf_class_name.pm") {
      my $code = "";
      print "/* Executing ...scripts/nios2.h/gtf/$gtf_class_name.pm */\n";
      $code .= "require \"$dir/nios2.h/BasicModule.pm\";";
      $code .= "require \"$dir/nios2.h/gtf/$gtf_class_name.pm\";";
      $code .= $gtf_class_name . "::run(\$system, \$no_legacy_module_name);";
      eval $code;
      if ($@) {
        print "#warning Could not execute ...scripts/nios2_system.h/gtf/$gtf_class_name.pm\n";
        print "#warning Error message is stored in nios2_system.h:\n";
        print "/*\n";
        print "$@";
        print "*/\n";
        print STDERR "Could not execute ...scripts/nios2_system.h/gtf/$gtf_class_name.pm\n";
        print STDERR "Error message follows:\n";
        print STDERR "$@";
} 
      last;
}   
}

}

}


1;
