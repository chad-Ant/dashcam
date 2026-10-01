id() { [ "${1:-}" = -u ] && echo 0 || command id "$@"; }
systemctl() { [ "${1:-}" = is-active ] && return 3; command systemctl "$@"; }
apt-get() { case " $* " in *" -s "*) command apt-get "$@" ;; *) echo "STUB apt-get $(wc -w <<< "$*") args" ;; esac; }
install() { echo "STUB install $*"; }
depmod() { echo "STUB depmod $*"; }
sed() { [ "${1:-}" = -i ] && { echo "STUB sed -i ${*: -1}"; return 0; }; command sed "$@"; }
export -f id systemctl apt-get install depmod sed
apt-mark() { if [ "${1:-}" = hold ]; then echo "STUB apt-mark hold: $(($# - 1)) packages"; return 0; fi; command apt-mark "$@"; }
export -f apt-mark
