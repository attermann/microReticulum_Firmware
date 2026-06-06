#!/usr/bin/env bash
#
# Package webconsole/index.html into a smaller delivery artifact.
#
# Modes:
#   --strip   (default) Remove the ?selftest=1 block; strip comments and
#             whitespace from HTML/CSS/JS via html-minifier-terser. Does
#             NOT mangle JS identifiers — output stays readable.
#   --minify  Same as --strip plus full JS mangling and CSS minification.
#
# Output:
#   Release/console.html by default. Override with --out PATH.
#
# Dependencies:
#   - bash, sed, awk (standard)
#   - npx + Node.js (for html-minifier-terser, fetched on first run and
#     cached by npm; no global install required)
#
# Source is never modified. The packaging script reads the source, slices
# the self-test block via SELFTEST-BEGIN / SELFTEST-END sentinels, then
# pipes through html-minifier-terser with mode-specific flags.

set -euo pipefail

# ---- defaults ----
mode="strip"
out_default_rel="Release/console.html"
out=""

# ---- arg parsing ----
while [[ $# -gt 0 ]]; do
  case "$1" in
    --strip)    mode="strip"; shift ;;
    --minify)   mode="minify"; shift ;;
    --out)      out="$2"; shift 2 ;;
    -h|--help)
      cat <<EOF
Usage: $0 [--strip | --minify] [--out PATH]

  --strip   (default) Comment + whitespace strip; preserves JS identifiers.
  --minify  Full minification + JS identifier mangling via terser.
  --out     Output path. Defaults to Release/console.html relative to the
            repo root (one level above this script).
EOF
      exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Run with --help for usage." >&2
      exit 2 ;;
  esac
done

# ---- resolve paths ----
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
src="$script_dir/index.html"

if [[ -z "$out" ]]; then
  out="$repo_root/$out_default_rel"
fi

if [[ ! -f "$src" ]]; then
  echo "Source not found: $src" >&2
  exit 1
fi

# ---- dep check ----
if ! command -v npx >/dev/null 2>&1; then
  echo "npx required (install Node.js: https://nodejs.org/)" >&2
  exit 1
fi

# ---- prepare output dir ----
mkdir -p "$(dirname "$out")"

# ---- slice SELFTEST block ----
# Removes every line from SELFTEST-BEGIN through SELFTEST-END inclusive.
tmp_sliced="$(mktemp -t console-sliced.XXXXXX.html)"
trap 'rm -f "$tmp_sliced"' EXIT

if ! grep -q "SELFTEST-BEGIN" "$src"; then
  echo "Warning: SELFTEST-BEGIN marker not found in source — self-test block will not be sliced." >&2
  cp "$src" "$tmp_sliced"
else
  sed '/SELFTEST-BEGIN/,/SELFTEST-END/d' "$src" > "$tmp_sliced"
fi

# ---- mode-specific html-minifier-terser flags ----
# Pinned to a tested version for reproducibility; bump deliberately.
hmt_pkg="html-minifier-terser@7.2.0"

# Common flags: strip HTML comments, collapse whitespace, minify inline CSS.
common_flags=(
  --collapse-whitespace
  --remove-comments
  --minify-css true
)

if [[ "$mode" == "strip" ]]; then
  # Strip JS comments and dead code via terser's compress pass, but keep
  # identifier names. Output stays readable for delivery-artifact debugging.
  js_cfg='{"compress":true,"mangle":false,"format":{"comments":false}}'
else
  # Full minification: terser defaults (compress + mangle), comments dropped.
  js_cfg='{"compress":true,"mangle":true,"format":{"comments":false}}'
fi

# ---- run minifier ----
# html-minifier-terser reads from stdin when no file arg is given, but its
# CLI works most reliably with a file argument. Feed the sliced temp file.
npx --yes "$hmt_pkg" \
  "${common_flags[@]}" \
  --minify-js "$js_cfg" \
  "$tmp_sliced" > "$out"

# ---- size summary ----
src_bytes="$(wc -c < "$src" | tr -d ' ')"
sliced_bytes="$(wc -c < "$tmp_sliced" | tr -d ' ')"
out_bytes="$(wc -c < "$out" | tr -d ' ')"

# Use awk for the percentage to avoid bc dependency.
pct="$(awk -v s="$src_bytes" -v o="$out_bytes" 'BEGIN{printf "%.1f", (s-o)*100.0/s}')"

printf '%s\n' "----- webconsole package summary -----"
printf '  mode:        %s\n'        "$mode"
printf '  source:      %8s bytes  %s\n' "$src_bytes"    "$src"
printf '  after slice: %8s bytes  (SELFTEST block removed)\n' "$sliced_bytes"
printf '  output:      %8s bytes  %s\n' "$out_bytes"    "$out"
printf '  saved:       %s%%  vs source\n' "$pct"
