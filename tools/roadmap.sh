#!/bin/sh
# Generate or check ROADMAP.md from docs/roadmap.sexp, using nova-tools'
# tools/roadmap at the pinned version below. Run from the repository root.
set -e
VERSION=v1.5.1-0.20261017123959-da3d50a06fba
pairs="docs/roadmap.sexp|ROADMAP.md"

cd "$(dirname "$0")/.."

case "${1:-}" in
  generate)
    for pair in $pairs; do
      data=${pair%%|*}
      page=${pair#*|}
      GOPROXY=direct go run github.com/mas-bandwidth/nova-tools/tools/roadmap@$VERSION --file "$data" --out "$page"
    done
    ;;
  check)
    fail=0
    for pair in $pairs; do
      data=${pair%%|*}
      page=${pair#*|}
      tmp=$(mktemp)
      GOPROXY=direct go run github.com/mas-bandwidth/nova-tools/tools/roadmap@$VERSION --file "$data" --out "$tmp" >/dev/null
      if ! cmp -s "$tmp" "$page"; then
        echo "ROADMAP.md is stale: run sh tools/roadmap.sh generate" >&2
        fail=1
      fi
      rows=$(grep -cE '^\| .+ \| (✅|❌) \| ' "$page")
      items=$(grep -c '^ *(item "' "$data")
      echo "ok $page rows=$rows items=$items"
      if [ "$rows" != "$items" ]; then
        echo "ROADMAP.md row count $rows does not match item count $items: run sh tools/roadmap.sh generate" >&2
        fail=1
      fi
      rm -f "$tmp"
    done
    [ "$fail" = 0 ] || exit 1
    ;;
  *)
    echo "usage: sh tools/roadmap.sh generate|check" >&2
    exit 2
    ;;
esac
