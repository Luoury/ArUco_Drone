#!/usr/bin/env bash
# 统一入口：一条命令跑任意一套验收流程。
#
#   bash scripts/run_flow.sh F1     # 纯手动（起飞 + 键盘航线 + 降落）
#   bash scripts/run_flow.sh F2     # 手动飞到码附近 → 按 2 → 视觉降落
#   bash scripts/run_flow.sh F3     # 纯自主航点
#   bash scripts/run_flow.sh F4     # 全自主（航点 → 搜码 → 降落）
#   bash scripts/run_flow.sh F5     # 异常处理（下降中遮挡 3 s）
#
# 每个流程都会：重启 SITL（带 IMU 健康检查）→ 起对应 launch → 起验收脚本 →
# 打印 [FLOW Fx] 判据明细与 RESULT → 收尾停掉所有进程。退出码 0 = PASS。
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLOW="${1:-}"
case "$FLOW" in
  F1|f1) FLOW=F1 ;;
  F2|f2) FLOW=F2 ;;
  F3|f3) FLOW=F3 ;;
  F4|f4) FLOW=F4 ;;
  F5|f5) FLOW=F5 ;;
  *)
    echo "用法: bash scripts/run_flow.sh <F1|F2|F3|F4|F5>" >&2
    exit 2
    ;;
esac

RUNNER="$ROOT/scripts/run_$(echo "$FLOW" | tr 'A-Z' 'a-z').sh"
if [ ! -f "$RUNNER" ]; then
  echo "找不到流程脚本: $RUNNER" >&2
  exit 2
fi

echo "=== run_flow: $FLOW ==="
bash "$RUNNER"
rc=$?
echo "=== run_flow: $FLOW 退出码 $rc（0 = PASS）==="
exit "$rc"
