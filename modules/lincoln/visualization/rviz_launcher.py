"""按车辆命名空间生成独立 RViz 配置，不改写用户模板。"""
from pathlib import Path
import re
import subprocess
import yaml
from modules.lincoln.runtime.paths import PROJECT_DIR, RUNTIME_DIR, RVIZ_BINARY


def adapt_rviz_namespace(value, uid, display_sensors=()):
    if isinstance(value, str):
        return re.sub(r'(?<![A-Za-z0-9_])lincoln_[0-9a-f]{10}(?=/|$)', uid, value)
    if isinstance(value, list):
        return [adapt_rviz_namespace(item, uid, display_sensors) for item in value]
    if isinstance(value, dict):
        result = {adapt_rviz_namespace(key, uid): adapt_rviz_namespace(item, uid, display_sensors)
                  for key, item in value.items()}
        topic = result.get('Topic')
        if isinstance(topic, dict) and str(topic.get('Value', '')).startswith(f'/{uid}/'):
            if topic['Value'].endswith(('/points', '/display_points', '/image', '/camera_info', '/cameras/mosaic')):
                topic['Reliability Policy'] = 'Best Effort'
            if (result.get('Class') == 'rviz_default_plugins/PointCloud2'
                    and topic['Value'] in {f'/{uid}/{name}/points' for name in display_sensors}):
                topic['Value'] = topic['Value'].removesuffix('/points') + '/display_points'
        if 'Class' in result and 'Enabled' in result:
            # RViz 先加载 Value，再加载 Topic/QoS，最后按 Enabled 启用显示。
            # 避免 Value=true 提前创建默认 Reliable 订阅，保留最终 Enabled 状态。
            result['Value'] = False
        return result
    return value


def launch_vehicle_rviz(node):
    template = node.args.rviz_config.expanduser()
    if not template.is_absolute():
        template = PROJECT_DIR / template
    saved = yaml.safe_load(template.read_text())
    if not isinstance(saved, dict) or 'Visualization Manager' not in saved:
        raise ValueError(f'无效 RViz 配置：{template}')
    folder = RUNTIME_DIR / node.uid
    folder.mkdir(parents=True, exist_ok=True)
    config = folder / 'lincoln.rviz'
    display_sensors = [entry['spec']['name'] for entry in node.sensors if 'display_pub' in entry]
    saved = adapt_rviz_namespace(saved, node.uid, display_sensors)
    view = saved['Visualization Manager'].get('Views', {}).get('Current', {})
    if view.get('Target Frame') == node.frame():
        view['Target Frame'] = node.model.header.frame_id
    config.write_text(yaml.safe_dump(saved,
                                     sort_keys=False, allow_unicode=True))
    log = folder / 'rviz.log'
    with log.open('a') as output:
        return subprocess.Popen([RVIZ_BINARY, '-d', str(config)], stdout=output, stderr=subprocess.STDOUT)
