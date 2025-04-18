#!/bin/bash

### ========== 🛠️ 可配置参数部分 ==========
SRC_FOLDER="/home/uav/water_swarm/src/swarm_planner/bspline_traj/traj/test"
USERNAME="vslam"
DEST_FOLDER="/home/${USERNAME}"
PASSWORD="1"
START_INDEX=0
END_INDEX=9
IP_PREFIX="192.168.3."
### =======================================

# 检查 sshpass 是否已安装
if ! command -v sshpass &> /dev/null; then
    echo "❌ 请先安装 sshpass： sudo apt install sshpass"
    exit 1
fi

declare -a pids

for ((i=START_INDEX; i<=END_INDEX; i++)); do
    FILE="${SRC_FOLDER}/${i}.txt"
    IP="${IP_PREFIX}$((100 + i))"

    if [ ! -f "$FILE" ]; then
        echo "❌ 文件不存在：${FILE}"
        continue
    fi

    echo "📤 [${i}] 正在将 ${FILE} 发送到 ${IP}:${DEST_FOLDER} ..."

    (
        sshpass -p "$PASSWORD" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            "$FILE" "${USERNAME}@${IP}:${DEST_FOLDER}/"

        if [ $? -eq 0 ]; then
            echo "✅ [${i}] 成功：${FILE} 已发送到 ${IP}"

            echo "🔄 [${i}] 正在 sync 并重启设备 ${IP} ..."

            # 🔧 在远程 shell 中先执行 sync，再执行 reboot（确保 sync 成功后才 reboot）
            sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
                ${USERNAME}@${IP} "echo $PASSWORD | sudo -S bash -c 'sync && echo sync_done && poweroff'" | \
                grep -q "sync_done"

            if [ $? -eq 0 ]; then
                echo "✅ [${i}] sync 成功，已触发重启 ${IP}"
            else
                echo "❌ [${i}] sync 或 reboot 出现异常"
            fi
        else
            echo "❌ [${i}] 失败：${FILE} 无法发送到 ${IP}"
        fi
    ) &

    pids+=($!)
done

# 等待所有任务结束
for pid in "${pids[@]}"; do
    wait $pid
done

echo "🎉 所有任务完成（包含发送 + sync + 重启）"
