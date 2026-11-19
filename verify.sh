#!/usr/bin/env bash
#
# 读取目标板运行态寄存器做验证（J-Link SWD，只读，不写 Flash）。用法：./verify.sh
#
# flash_cli.jlink / verify_cli.jlink 里的 loadfile 与 J-Link 都按"仓库根为 CWD"
# 解释相对路径，本脚本会自己 cd 到仓库根，所以在任何目录下调用都可以。
#
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# JLinkExe 查找顺序：PATH -> /opt/SEGGER/JLink*/ -> $HOME/opt/SEGGER/JLink*/
find_jlink() {
    local p
    if p="$(command -v JLinkExe 2>/dev/null)"; then
        printf '%s\n' "$p"
        return 0
    fi
    for p in /opt/SEGGER/JLink*/JLinkExe ${HOME:+"$HOME"/opt/SEGGER/JLink*/JLinkExe}; do
        if [[ -x "$p" ]]; then
            printf '%s\n' "$p"
            return 0
        fi
    done
    return 1
}

if ! JLINK="$(find_jlink)"; then
    cat >&2 <<'EOF'
找不到 JLinkExe（J-Link Commander）。先装 SEGGER J-Link 软件包：
  1) 下载 Linux 版 "J-Link Software and Documentation Pack"
     https://www.segger.com/downloads/jlink/
  2) sudo dpkg -i JLink_Linux_V*.deb      # Debian/Ubuntu，默认装到 /opt/SEGGER/JLink_<版本>/
  3) 确认 /opt/SEGGER/JLink_<版本>/JLinkExe 存在且可执行；若不在 PATH 里就加进去：
       export PATH="/opt/SEGGER/JLink_V914a:$PATH"
USB 权限：报 "Cannot connect to J-Link" 时按 SEGGER 说明装 udev 规则
（软件包自带 99-jlink.rules），或临时 sudo 运行。
EOF
    exit 1
fi

cd "$REPO_ROOT"

exec "$JLINK" -CommanderScript verify_cli.jlink -AutoConnect 1 -ExitOnError 1
