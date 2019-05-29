ping 8.8.8.8 | gawk -F "[ =]" '{print systime(),$10}' | feedgnuplot --stream --exit --domain --timefmt %s --lines

ping 8.8.8.8 | gawk -F "[ =]" '/^64 bytes/ {print systime(),$10}' |feedgnuplot --stream --exit --domain --timefmt %s --lines

ping -A -D 8.8.8.8 |
  perl -anE 'BEGIN { $| = 1; }
             $F[0] =~ s/[\[\]]//g or next;
             $F[7] =~ s/.*=//g    or next;
             say "$F[0] $F[7]"' |
  feedgnuplot --stream --domain --histogram 0 --binwidth 10 \
              --xlabel 'Ping round-trip time (s)'  \
              --ylabel Frequency --xlen 20


ping 8.8.8.8 | unbuffer -p cut -d ' ' -f 7 | unbuffer -p cut -d '=' -f 2 | feedgnuplot --lines --stream --terminal 'dumb 80,24' --xlen 200 --extracmds 'unset grid'

ping 8.8.8.8 | unbuffer -p cut -d ' ' -f 7 | unbuffer -p cut -d '=' -f 2 | feedgnuplot --lines --stream --xlen 200 --extracmds 'unset grid'

ping 172.31.32.232 | unbuffer -p awk 'BEGIN {FS="[=]|[ ]"} {print $10}' | feedgnuplot --lines --stream --xlen 200 --extracmds 'unset grid'
