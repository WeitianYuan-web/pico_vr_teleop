#!/usr/bin/env bash
# Block until official proc_manager has left Initing and body.launch is up.
set +u
LOG_TAG="[wait-proc-manager]"
MAX_S="${TIANYEE_WAIT_PROC_MANAGER_S:-180}"

echo "$LOG_TAG waiting up to ${MAX_S}s for proc_manager Running + body.launch ..."
deadline=$((SECONDS + MAX_S))
while (( SECONDS < deadline )); do
  if ! systemctl is-active --quiet proc_manager.service; then
    echo "$LOG_TAG proc_manager not active yet"
    sleep 2
    continue
  fi
  if ! pgrep -f 'body.launch.py' >/dev/null; then
    echo "$LOG_TAG waiting body.launch.py ..."
    sleep 2
    continue
  fi
  # Prefer journal evidence of Running; fall back once body has been up a bit.
  if journalctl -u proc_manager.service -n 80 --no-pager 2>/dev/null \
      | grep -q 'self-check passed, current status: Running'; then
    echo "$LOG_TAG OK: proc_manager reports Running"
    # small settle delay so TTS/self-check UI can finish
    sleep 5
    exit 0
  fi
  # body up for a while is enough fallback
  body_age=$(ps -o etimes= -p "$(pgrep -f 'body.launch.py' | head -1)" 2>/dev/null | tr -d ' ')
  if [[ -n "$body_age" && "$body_age" -ge 25 ]]; then
    echo "$LOG_TAG OK: body.launch age=${body_age}s (journal Running not seen)"
    sleep 3
    exit 0
  fi
  echo "$LOG_TAG still waiting (body_age=${body_age:-0}s) ..."
  sleep 3
done
echo "$LOG_TAG WARN: timeout — starting XARM anyway"
exit 0
