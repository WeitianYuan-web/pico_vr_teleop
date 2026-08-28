# 本仓 IO 手型约定。由 run_io_gateway / run_io_zenoh2ros / run_hand_controller source。
# 覆盖：export IO_HAND_MODEL=rh5dg2   或   IO_HANDS=Inspire_RH5DG2

io_hand_normalize_model() {
  local raw
  raw="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
  raw="${raw%/}"
  case "${raw}" in
    rh5dg2|g2|dg2|inspire_rh5dg2|064) printf '%s' "rh5dg2" ;;
    rh56f2|f2|inspire_rh56f2|"") printf '%s' "rh56f2" ;;
    *) printf '%s' "${raw}" ;;
  esac
}

io_hand_resolve_model() {
  local raw="${IO_HAND_MODEL:-}"
  if [[ -z "${raw}" && -n "${IO_HANDS:-}" ]]; then
    raw="${IO_HANDS}"
  fi
  io_hand_normalize_model "${raw}"
}

io_hand_io_name() {
  case "$(io_hand_normalize_model "${1:-$(io_hand_resolve_model)}")" in
    rh5dg2) printf '%s' "Inspire_RH5DG2" ;;
    *) printf '%s' "Inspire_RH56F2" ;;
  esac
}
