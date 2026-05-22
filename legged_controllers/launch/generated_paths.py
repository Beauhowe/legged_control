"""Generated artifacts: URDF, controller yaml, OCS2 CppAD libs under repo legged_control/."""

import os
import re


def get_repo_root() -> str:
    if repo := os.environ.get("LEGGED_CONTROL_REPO"):
        return os.path.abspath(repo)

    src_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if os.path.isdir(os.path.join(src_repo, "legged_control")):
        return src_repo

    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix("legged_controllers")
        candidate = os.path.join(os.path.abspath(os.path.join(prefix, "..", "..")), "src", "legged_control")
        if os.path.isdir(os.path.join(candidate, "legged_control")):
            return candidate
    except Exception:
        pass

    return src_repo


def get_generated_dir() -> str:
    generated_dir = os.path.join(get_repo_root(), "legged_control")
    os.makedirs(generated_dir, exist_ok=True)
    return generated_dir


def get_cppad_model_folder(robot_type: str) -> str:
    folder = os.path.join(get_generated_dir(), robot_type)
    os.makedirs(folder, exist_ok=True)
    return folder


def resolve_task_file(task_file: str, robot_type: str) -> str:
    """Copy task.info with modelFolderCppAd pointing at legged_control/{robot_type}."""
    generated_dir = get_generated_dir()
    dst = os.path.join(generated_dir, f"{robot_type}_task.info")
    model_folder = get_cppad_model_folder(robot_type)
    with open(task_file, encoding="utf-8") as src:
        content = src.read()
    content = re.sub(
        r"^(\s*modelFolderCppAd\s+).*$",
        rf"\1{model_folder}",
        content,
        count=1,
        flags=re.MULTILINE,
    )
    with open(dst, "w", encoding="utf-8") as out:
        out.write(content)
    return dst
