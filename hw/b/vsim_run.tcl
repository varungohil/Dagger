# Remove "Memory" from WildcardFilter so that unpacked
# arrays will be included in the collected data.
quietly set WildcardFilter [lsearch -not -all -inline $WildcardFilter Memory]

add wave -r /* 
run -all
