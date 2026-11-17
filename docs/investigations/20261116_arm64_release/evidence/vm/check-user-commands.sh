set -u
runuser -u orangepi -- bash /home/orangepi/install_fc01.sh --status
rc=$?
printf '\nSTATUS_COMMAND_EXIT=%s\n' "$rc"
runuser -u orangepi -- python3 /home/orangepi/FC01_versions/20261116_stage3/navigation.py status
rc=$?
printf '\nNAVIGATION_STATUS_EXIT=%s\n' "$rc"
runuser -u orangepi -- bash -c 'FC01_BACKUP=$(/usr/bin/python3 -c '\''import json,os; print(json.load(open(os.path.expanduser("~/FC01_versions/20261116_stage3/deployment/INSTALL_STATE.json")))["backup"])'\''); bash /home/orangepi/install_fc01.sh --rollback "$FC01_BACKUP"'
rc=$?
printf '\nSECOND_ROLLBACK_EXPECTED_REFUSAL_EXIT=%s\n' "$rc"
