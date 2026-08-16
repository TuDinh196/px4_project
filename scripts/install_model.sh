#!/usr/bin/env bash
# Backward compatibility wrapper for manage.sh setup
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$PROJECT_DIR/manage.sh" setup "$@"
