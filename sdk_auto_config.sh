#!/usr/bin/expect -f
spawn python3.6 -m anki_vector.configure -e $env(ANKI_USER_EMAIL) -n $env(VECTOR_NAME) -i $env(VECTOR_IP) -s $env(VECTOR_SERIAL)
expect "Do you wish to proceed? \(y/n\) "
send "y\n"
expect "Enter Password: "
send "$env(ANKI_USER_PASSWORD)\r"
expect "SUCCESS!"