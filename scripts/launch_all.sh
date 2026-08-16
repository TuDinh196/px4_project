#!/usr/bin/env bash
# Backward compatibility wrapper for manage.sh all
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$PROJECT_DIR/manage.sh" all "$@"
