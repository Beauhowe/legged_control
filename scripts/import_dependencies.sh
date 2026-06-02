#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
src_dir="$(cd "${repo_root}/.." && pwd)"
repos_file="${repo_root}/dependencies.repos"

if ! command -v vcs >/dev/null 2>&1; then
  echo "error: vcs is not installed. Install vcstool first." >&2
  exit 1
fi

if [[ ! -f "${repos_file}" ]]; then
  echo "error: missing ${repos_file}" >&2
  exit 1
fi

echo "Importing dependencies into ${src_dir}"
vcs import "${src_dir}" < "${repos_file}"

if [[ -d "${src_dir}/pinocchio/.git" || -f "${src_dir}/pinocchio/.git" ]]; then
  echo "Initializing pinocchio submodules"
  git -C "${src_dir}/pinocchio" submodule update --init --recursive
else
  echo "warning: ${src_dir}/pinocchio is missing; skip submodule initialization" >&2
fi

cd "${src_dir}"/hpp-fcl
git submodule update --init --recursive

echo "Dependencies are ready."
