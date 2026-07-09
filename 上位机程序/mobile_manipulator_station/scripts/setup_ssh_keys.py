#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None


def load_config(config_path: Path):
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，请先安装：python3 -m pip install PyYAML")
    if not config_path.exists():
        raise FileNotFoundError(f"未找到配置文件: {config_path}")
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    root = data.get("ssh_targets", {})
    key_type = str(root.get("key_type", "ed25519")).strip() or "ed25519"
    targets = root.get("targets", []) or []
    valid_targets = []
    for item in targets:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name", "")).strip() or "unnamed"
        host = str(item.get("host", "")).strip()
        if host:
            valid_targets.append({"name": name, "host": host})
    if not valid_targets:
        raise RuntimeError("配置文件里没有可用的 SSH 目标。")
    return key_type, valid_targets


def ensure_local_key(key_type: str) -> None:
    ssh_dir = Path.home() / ".ssh"
    ssh_dir.mkdir(parents=True, exist_ok=True)
    private_key = ssh_dir / f"id_{key_type}"
    public_key = ssh_dir / f"id_{key_type}.pub"
    if private_key.exists() and public_key.exists():
        print(f"[OK] 已存在本机密钥: {private_key}")
        return

    print(f"[INFO] 正在生成本机 SSH 密钥: {private_key}")
    subprocess.run(
        ["ssh-keygen", "-t", key_type, "-f", str(private_key), "-N", ""],
        check=True,
    )


def copy_key_to_target(host: str) -> None:
    print(f"[INFO] 正在配置免密登录: {host}")
    subprocess.run(["ssh-copy-id", host], check=True)
    print(f"[OK] 免密登录已写入: {host}")


def test_target(host: str) -> None:
    print(f"[INFO] 正在测试连接: {host}")
    completed = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, "echo ok"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or completed.stdout.strip() or "未知错误"
        raise RuntimeError(f"连接测试失败: {host}\n{stderr}")
    print(f"[OK] 连接测试通过: {host}")


def main() -> int:
    parser = argparse.ArgumentParser(description="按配置文件批量配置 SSH 免密登录")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent.parent / "config" / "ssh_targets.yaml"),
        help="SSH 目标配置文件路径",
    )
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="只写入公钥，不做连接测试",
    )
    args = parser.parse_args()

    try:
        key_type, targets = load_config(Path(args.config))
        ensure_local_key(key_type)
        for target in targets:
            print(f"\n=== {target['name']} | {target['host']} ===")
            copy_key_to_target(target["host"])
            if not args.skip_test:
                test_target(target["host"])
    except KeyboardInterrupt:
        print("\n[WARN] 用户中断。")
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1

    print("\n[OK] 全部 SSH 目标已处理完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
