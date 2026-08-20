import re

from fleet_bridge_config.models import TopicConfig


def _exact_topic_pattern(topic: str) -> str:
    return f'^{re.escape(topic)}$'


def _unique(items):
    return list(dict.fromkeys(items))


def filtered_topics(topics: tuple[TopicConfig, ...]) -> tuple[TopicConfig, ...]:
    return tuple(
        topic
        for topic in topics
        if topic.enabled and topic.filter.mode != 'passthrough'
    )


def forwarded_topics(topics: tuple[TopicConfig, ...]) -> tuple[TopicConfig, ...]:
    """Return enabled topics that the gateway must republish locally.

    A passthrough policy controls rate and payload handling, not whether a
    topic is renamed. Root vehicle topics must still be relayed to their
    vehicle-specific namespace before Foxglove advertises them to the server.
    """

    return tuple(
        topic
        for topic in topics
        if topic.enabled and topic.source != topic.uplink
    )


def bridge_parameters(
    topics: tuple[TopicConfig, ...],
    mode: str,
    port: int,
) -> dict[str, object]:
    if mode not in {'fleet', 'debug'}:
        raise ValueError('mode must be fleet or debug')
    if port < 1 or port > 65535:
        raise ValueError('port must be between 1 and 65535')

    if mode == 'fleet':
        selected = tuple(topic for topic in topics if topic.enabled)
        names = [topic.uplink for topic in selected]
    else:
        selected = tuple(topic for topic in topics if topic.debug)
        names = [topic.source for topic in selected]

    best_effort_names = [
        topic.uplink if mode == 'fleet' else topic.source
        for topic in selected
        if topic.qos.reliability == 'best_effort'
    ]
    return {
        'address': '0.0.0.0',
        'port': port,
        'topic_whitelist': [_exact_topic_pattern(name) for name in _unique(names)],
        'best_effort_qos_topic_whitelist': [
            _exact_topic_pattern(name) for name in _unique(best_effort_names)
        ],
        'service_whitelist': ['(?!)'],
        'param_whitelist': ['(?!)'],
        'client_topic_whitelist': ['(?!)'],
        'asset_uri_allowlist': ['(?!)'],
        # ROS 2 Humble cannot encode an empty string-array override. The pinned
        # bridge enables capabilities only by exact match with known names.
        'capabilities': ['none'],
        'include_hidden': False,
        'min_qos_depth': 1,
        'max_qos_depth': 20,
        'send_buffer_limit': 4 * 1024 * 1024,
        'use_compression': False,
    }
